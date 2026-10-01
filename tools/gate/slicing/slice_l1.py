#!/usr/bin/env python3
"""L1「可打印」的**真切片**取数脚本 —— 离线跑，把指标固化进 data/printability.yaml。

为什么不在 Gate 层里直接调切片器：
  · 切片器是外部可执行文件（09-08 那次从 /tmp 临时挂载的 DMG 跑，关机就没了；09-13 起
    正式装在 /Applications/PrusaSlicer.app）。Gate 每次都必须能跑，不能依赖它在不在。
  · 所以本脚本把「切片这一步」做成有出处、可复现的**离线取数**：
    输入 = cad/duck_s288/*.stl（导出件本身，不是 placed/ 的世界坐标副本，元规则 1），
    输出 = data/printability.yaml:slice_run 里逐件的指标 + **源 STL 的 sha256**。
    层每次跑只核对「当前 STL 的 sha256 == 记录里的 source_sha256」；对不上就判 STALE/红。
  · G-code 本体 75 MB，不入库；写到 --gcode-dir（默认系统临时目录）。指标已固化，
    要复核就重跑本脚本。

支撑落点（09-13，审计 F-L1-3 落地版）：
  · 解析 ;TYPE:Support material / Support material interface 的挤出段（G1 带 E 且 XY 有位移；;TYPE 在层间不重发，
    类型跟着挤出记入当前层），沿每段每 criteria.support_landing_sample_step_mm（0.2）取一点（**含端点**），
    点的 z = 该层 ;Z:（层顶；不另取层底）。
  · 逆变换回 export_local：切片前的变换是 M = align_vectors(down_local → (0,0,-1))、再平移 shift 到 (100,100,0)，
    所以 p_export_local = M⁻¹·(p_print − shift)。M 与 shift 记进 slice_run.parts.<件>.orient_transform 以便复算。
  · 禁撑区由 layers/l1_printable.no_support_zone_set()（printability.yaml:no_support_zones 两级 × features.yaml）建，
    与层同一函数、同一 zones_sha256；逐区统计落进去的点数与折算路径长（点数×步长）→ support_landing。
  · 逐件切片参数：printability.yaml:parts.<件>.slicer_args（例 N03 --support-material-buildplate-only）追加进 argv，
    实际追加的参数记进 slice_run.parts.<件>.slicer_argv_extra。G-code 仍不入库。
  · 逐件朝向 / 填充率 / 周界生成器也全在 printability.yaml:parts.<件>（down_world / fill_density_percent / perimeter_generator），
    本文件不再有件号表（09-13）。`--parts A,B` 只重切这几件，其余件沿用 slice_run.yaml 现有记录。

用法：
    ./.venv/bin/python tools/gate/slicing/slice_l1.py \
        --exe /Applications/PrusaSlicer.app/Contents/MacOS/PrusaSlicer     # 09-13 起的常驻安装；不给 --exe 就用这个
写出 tools/gate/slicing/slice_run.yaml（贴进 data/printability.yaml:slice_run）
和 tools/gate/slicing/logs/<件>.log、<件>_profile.ini。
"""
from __future__ import annotations
import argparse, hashlib, json, math, re, subprocess, sys, tempfile, time
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DATA = HERE.parent / "data"
for _p in (str(HERE.parent), str(HERE.parent / "layers")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import l1_printable as L1          # noqa: E402  禁撑区集合 no_support_zone_set / zone_hits（与层共用，sha 才对得上）
DEFAULT_EXE = "/Applications/PrusaSlicer.app/Contents/MacOS/PrusaSlicer"   # 09-13 常驻安装（2.9.6）

# 逐件的打印朝向（世界系朝下法向）/ 填充率 / 周界生成器 / 追加切片参数 —— 全部读 printability.yaml:parts.<件>
# （2026-09-13 起；以前是本文件里写死的朝向/填充率表、placed 名表和一个按件号切换周界生成器的分支 —— 换件必改代码，审计 §2.6）。
# 缺字段直接退出（返回码 4），不给缺省值：朝向缺省 = 拿错朝向切出一份"有出处"的假记录。
_PART_KEYS = ("down_world", "fill_density_percent")


def part_cfg(P: dict, pid: str) -> dict:
    """printability.yaml:parts[pid] → {down_world, fill_density_percent, perimeter_generator, slicer_args}。缺必填键抛 KeyError。"""
    c = (P.get("parts") or {}).get(pid) or {}
    for k in _PART_KEYS:
        if k not in c:
            raise KeyError(f"printability.yaml:parts.{pid}.{k} 缺失")
    dw = c["down_world"]
    dw = dw.get("v") if isinstance(dw, dict) else dw
    fd = c["fill_density_percent"]
    fd = fd.get("v") if isinstance(fd, dict) else fd
    if not (isinstance(dw, (list, tuple)) and len(dw) == 3 and all(isinstance(v, (int, float)) for v in dw)):
        raise KeyError(f"printability.yaml:parts.{pid}.down_world 不是三元向量：{dw!r}")
    return {"down_world": [float(v) for v in dw], "fill_density_percent": fd,
            "perimeter_generator": str(c.get("perimeter_generator") or "arachne"),
            "slicer_args": [str(x) for x in (c.get("slicer_args") or [])]}


def _canon(o):
    """hr42（第四任）：切片复用键的规范化 —— 数值全精度（numpy 数组按 dtype/shape/逐元素 repr），不用 str()（numpy 的 str 只留 8 位、
    大数组还会省略号，会让不同输入撞同一个键）。认不得的对象用 repr。"""
    if isinstance(o, dict):
        return {str(k): _canon(v) for k, v in sorted(o.items(), key=lambda kv: str(kv[0]))}
    if isinstance(o, (list, tuple)):
        return [_canon(v) for v in o]
    if isinstance(o, np.ndarray):
        return {"__nd__": str(o.dtype), "shape": list(o.shape), "v": [repr(x) for x in o.ravel().tolist()]}
    if isinstance(o, np.generic):
        return repr(o.item())
    if isinstance(o, float):
        return repr(o)
    if o is None or isinstance(o, (bool, int, str)):
        return o
    if hasattr(o, "wkb_hex"):
        return {"__geom__": o.wkb_hex}
    return {"__repr__": repr(o)}


def _lib_versions():
    import importlib.metadata as md, platform
    out = [f"python={sys.version.split()[0]}", f"machine={platform.machine()}"]
    for n in ("numpy", "trimesh", "shapely", "PyYAML"):
        try:
            out.append(f"{n}={md.version(n)}")
        except Exception:
            out.append(f"{n}=-")
    return out


def placed_name(parts_yaml: list, pid: str):
    rec = next((r for r in parts_yaml if r.get("id") == pid), None) or {}
    inst = rec.get("placed_instances") or []
    return inst[0] if inst else None

SETTINGS = [
    "--layer-height", "0.2", "--first-layer-height", "0.2",
    "--nozzle-diameter", "0.4", "--filament-diameter", "1.75",
    "--filament-density", "1.27", "--perimeters", "3",
    "--perimeter-generator", "arachne", "--fill-pattern", "rectilinear",
    "--top-solid-layers", "4", "--bottom-solid-layers", "4",
    "--support-material", "--support-material-auto",
    "--support-material-threshold", "45", "--support-material-style", "snug",
    "--support-material-contact-distance", "0.2", "--support-material-xy-spacing", "0.3",
    "--brim-width", "3", "--gcode-comments", "--no-binary-gcode",
    "--gcode-flavor", "marlin", "--threads", "2",
    "--center", "100,100", "--dont-arrange",
]


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def down_local(pid: str, frames: dict, down_world) -> list[float]:
    """世界空间朝下法向 → 件自身（export_local）坐标系。
    R = features.yaml:frames.per_part[pid].world_to_export_local_R（p_export = R·p_world + t）。"""
    R = np.array(frames[pid]["world_to_export_local_R"], dtype=float)
    return [float(round(v, 9)) for v in (R @ np.array(down_world, dtype=float))]


def parse_gcode(text: str, mesh=None) -> dict:
    """从 gcode 里取真实指标。E 是绝对坐标（M82），按 ΔE × 丝截面积换算体积。

    09-12 第二批两处修正（I1 调查发现）：
      1. ;TYPE: 在层间不变时 PrusaSlicer **不重发**，旧解析只在看到 ;TYPE: 才记类型，每隔几层就把
         有墙的层记成"没墙"（L03 142/74 实为 169/101）。现在类型跟着挤出动作记入当前层。
      2. Arachne 把 2 条 bead 的墙（约 0.9..1.17 mm）也全标成 External perimeter（inset 0/0），
         "整层只有 External perimeter" ≠ "只走了 1 圈"。所以另算 **单 bead 层**：该层 External perimeter
         （含 Overhang perimeter —— 悬空边界段被切片器改标成它，仍是外圈 bead）走线总长 / 该层网格截面周长
         —— 每条边界面各有一条 bead 时比值 ≈ 1，整面墙只有一条中线 bead 时 ≈ 0.5。
         比值 < 0.75 记为 layers_single_bead（需要 mesh：切片时用的摆好的网格）。
    """
    area = math.pi * (1.75 / 2) ** 2          # 1.75 mm 丝，2.4053 mm²
    per_type_e: dict[str, float] = {}
    layer_types: list[set] = []
    layer_z: list[float] = []
    layer_h: list[float] = []
    layer_ext_len: list[float] = []
    cur_type, cur_e, cur_layer = "", 0.0, None
    x = y = None
    widths: dict[str, float] = {}
    header = {}
    for line in text.splitlines():
        if line.startswith(";"):
            m = re.match(r";\s*([a-z ]+) extrusion width = ([\d.]+)mm", line)
            if m:
                widths[m.group(1).strip()] = float(m.group(2))
            if line.startswith(";TYPE:"):
                cur_type = line[6:].strip()
            elif line.startswith(";LAYER_CHANGE"):
                if cur_layer is not None:
                    layer_types.append(cur_layer)
                cur_layer = set()
                layer_z.append(float("nan")); layer_h.append(float("nan")); layer_ext_len.append(0.0)
            elif line.startswith(";Z:") and layer_z:
                layer_z[-1] = float(line[3:])
            elif line.startswith(";HEIGHT:") and layer_h:
                layer_h[-1] = float(line[8:])
            for key in ("filament used [mm]", "filament used [cm3]", "filament used [g]",
                        "estimated printing time (normal mode)"):
                if line.startswith("; " + key + " = "):
                    header[key] = line.split(" = ", 1)[1].strip()
            continue
        if line.startswith("G92") and " E0" in line:
            cur_e = 0.0
            continue
        if line.startswith(("G1 ", "G0 ")):
            mx = re.search(r"\bX(-?[\d.]+)", line)
            my = re.search(r"\bY(-?[\d.]+)", line)
            nx = float(mx.group(1)) if mx else x
            ny = float(my.group(1)) if my else y
            m = re.search(r"\bE(-?[\d.]+)", line)
            if m:
                e = float(m.group(1))
                d = e - cur_e
                cur_e = e
                if d > 0 and (mx or my):
                    per_type_e[cur_type] = per_type_e.get(cur_type, 0.0) + d
                    if cur_layer is not None:
                        cur_layer.add(cur_type)                    # 类型跟着挤出记入本层（修正 1）
                        if cur_type in ("External perimeter", "Overhang perimeter") and x is not None and y is not None and nx is not None and ny is not None:
                            layer_ext_len[-1] += math.hypot(nx - x, ny - y)
            x, y = nx, ny
    if cur_layer is not None:
        layer_types.append(cur_layer)
    vol = {k: round(v * area, 3) for k, v in sorted(per_type_e.items())}
    peri_idx = [i for i, t in enumerate(layer_types) if ("Perimeter" in t or "External perimeter" in t)]
    ext_only_idx = [i for i in peri_idx if "External perimeter" in layer_types[i] and "Perimeter" not in layer_types[i]]
    single_bead, ratios, unmeasured = [], [], 0
    if mesh is not None:
        for i in ext_only_idx:
            z, h = layer_z[i], layer_h[i]
            if not (math.isfinite(z) and math.isfinite(h)):
                unmeasured += 1
                continue
            try:
                sec = mesh.section(plane_origin=[0.0, 0.0, z - h / 2.0], plane_normal=[0.0, 0.0, 1.0])
            except Exception:                                      # noqa: BLE001
                sec = None
            if sec is None or not len(sec.entities):
                unmeasured += 1
                continue
            per = float(sum(e.length(sec.vertices) for e in sec.entities))
            if per <= 0:
                unmeasured += 1
                continue
            r = layer_ext_len[i] / per
            ratios.append(round(r, 3))
            if r < 0.75:
                single_bead.append(i)
    return {
        "layers": len(layer_types),
        "extrusion_width_mm": widths,
        "filament": header,
        "volume_mm3_by_type": vol,
        "support_volume_mm3": round(vol.get("Support material", 0.0)
                                    + vol.get("Support material interface", 0.0), 3),
        "overhang_perimeter_volume_mm3": vol.get("Overhang perimeter", 0.0),
        "bridge_infill_volume_mm3": vol.get("Bridge infill", 0.0),
        "gap_fill_volume_mm3": vol.get("Gap fill", 0.0),
        "layers_with_perimeter": len(peri_idx),
        "layers_external_perimeter_only": len(ext_only_idx),
        "layers_external_perimeter_only_note": "Arachne 下 2 条 bead 的墙也只标 External perimeter，此数不等于单圈墙层数；单圈看 layers_single_bead",
        "layers_single_bead": (None if mesh is None else len(single_bead)),
        "single_bead_ratio_threshold": 0.75,
        "single_bead_ratios_ext_only_layers": (None if mesh is None else ratios),
        "single_bead_unmeasured_layers": (None if mesh is None else unmeasured),
    }


SUPPORT_TYPES = ("Support material", "Support material interface")


def parse_support_segments(text: str) -> list[tuple[float, float, float, float, float]]:
    """G-code → 支撑挤出段 [(x0, y0, x1, y1, z)…]（打印坐标，z = 该层 ;Z:）。
    只取 ;TYPE ∈ SUPPORT_TYPES 且 G1 带正 ΔE、XY 有位移的段；;TYPE 在层间不重发，类型沿用（与 parse_gcode 修正 1 同一口径）。
    E 绝对坐标（M82），G92 E0 复位。"""
    segs = []
    cur_type, cur_e, z = "", 0.0, float("nan")
    x = y = None
    for line in text.splitlines():
        if line.startswith(";"):
            if line.startswith(";TYPE:"):
                cur_type = line[6:].strip()
            elif line.startswith(";Z:"):
                try:
                    z = float(line[3:])
                except ValueError:
                    z = float("nan")
            continue
        if line.startswith("G92") and " E0" in line:
            cur_e = 0.0
            continue
        if line.startswith(("G1 ", "G0 ")):
            mx = re.search(r"\bX(-?[\d.]+)", line)
            my = re.search(r"\bY(-?[\d.]+)", line)
            nx = float(mx.group(1)) if mx else x
            ny = float(my.group(1)) if my else y
            m = re.search(r"\bE(-?[\d.]+)", line)
            if m:
                e = float(m.group(1)); d = e - cur_e; cur_e = e
                if (d > 0 and (mx or my) and cur_type in SUPPORT_TYPES and math.isfinite(z)
                        and x is not None and y is not None and nx is not None and ny is not None
                        and (nx != x or ny != y)):
                    segs.append((x, y, nx, ny, z))
            x, y = nx, ny
    return segs


def sample_segments(segs, step: float):
    """沿每段每 step 取一点，含两端点（段长 L → ceil(L/step)+1 点）。返回 (N×3 打印坐标, 路径总长 mm)。"""
    pts, total = [], 0.0
    for x0, y0, x1, y1, z in segs:
        L = math.hypot(x1 - x0, y1 - y0)
        total += L
        n = max(1, int(math.ceil(L / step)))
        t = np.linspace(0.0, 1.0, n + 1)
        pts.append(np.column_stack([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, np.full(n + 1, z)]))
    if not pts:
        return np.zeros((0, 3)), 0.0
    return np.vstack(pts), total


def support_landing(text: str, M, shift, zone_set: dict, step: float) -> dict:
    """G-code 文本 + 切片前变换 (M 4×4, shift 3) + 禁撑区集合 → support_landing 记录（写进 slice_run.parts.<件>）。
    per_zone 只列命中 > 0 的区（zones_n / zones_sha256 说明评了哪一套）。"""
    segs = parse_support_segments(text)
    P, total = sample_segments(segs, step)
    Minv = np.linalg.inv(np.asarray(M, dtype=float))
    Pl = (Minv[:3, :3] @ (P - np.asarray(shift, dtype=float)).T).T + Minv[:3, 3] if len(P) else P
    zones = zone_set["zones"]
    hits = L1.zone_hits(zones, Pl, zone_set["margin"] or 0.0)
    per = {}
    for zn in zones:
        n = hits.get(zn["tag"], 0)
        if n > 0:
            per[zn["tag"]] = {"samples": int(n), "path_mm": round(n * step, 2), "kind": zn["kind"],
                              "tier": zn["tier"], "feature": zn["feature"], "role": zn["role"]}
    tier_n = {"forbid": 0, "removable": 0}
    for zn in zones:
        tier_n[zn["tier"]] += hits.get(zn["tag"], 0)
    return {"sample_step_mm": step,
            "z_rule": "每点 z = 该层 ;Z:（层顶）；每段含两端点，相邻段的公共端点各计一次",
            "types": list(SUPPORT_TYPES),
            "segments": len(segs), "samples_total": int(len(P)), "support_path_mm": round(total, 1),
            "zones_n": len(zones), "zones_sha256": zone_set["sha256"],
            "zones_forbid_n": sum(1 for zn in zones if zn["tier"] == "forbid"),
            "zones_removable_n": sum(1 for zn in zones if zn["tier"] == "removable"),
            "hits_forbid_samples": tier_n["forbid"], "hits_removable_samples": tier_n["removable"],
            "per_zone": per}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=DEFAULT_EXE)
    ap.add_argument("--gcode-dir", default=str(Path(tempfile.gettempdir()) / "gate_l1_gcode"))
    ap.add_argument("--parts", default="")
    ap.add_argument("--jobs", type=int, default=0, help="hr42：并行切片器进程数（默认 SLICE_JOBS 或 4）")
    ap.add_argument("--no-check-cache", action="store_true", help="hr42：不沿用上次记录，全部重切")
    a = ap.parse_args(argv)

    exe = Path(a.exe)
    if not exe.exists():
        print(f"切片器不存在：{exe}", file=sys.stderr)
        return 3
    gdir = Path(a.gcode_dir); gdir.mkdir(parents=True, exist_ok=True)
    logs = HERE / "logs"; logs.mkdir(exist_ok=True)

    feats_doc = yaml.safe_load((DATA / "features.yaml").read_text(encoding="utf-8"))
    frames = feats_doc["frames"]["per_part"]
    feats_all = feats_doc.get("features") or []
    parts = yaml.safe_load((DATA / "parts.yaml").read_text(encoding="utf-8"))["parts"]
    P = yaml.safe_load((DATA / "printability.yaml").read_text(encoding="utf-8")) or {}
    step_field = (P.get("criteria") or {}).get("support_landing_sample_step_mm")
    step = float(step_field["v"]) if isinstance(step_field, dict) and step_field.get("v") is not None else None
    if step is None:
        print("printability.yaml:criteria.support_landing_sample_step_mm 缺失 —— 支撑落点无从采样", file=sys.stderr)
        return 4
    want = {x for x in a.parts.split(",") if x}
    stl_index = {p.stem.split("_")[0]: p for p in sorted((ROOT / "cad/duck_s288").glob("*.stl"))}

    out = {}
    if want:
        # 子集重切：其余件沿用现有 slice_run.yaml 的记录（不许把 18 件的记录覆盖成 1 件）；
        # 沿用的记录仍带各自的 source_sha256 / zones_sha256，层照旧逐件核对，过期照旧红。
        prev = (HERE / "slice_run.yaml")
        if prev.exists():
            out = dict(((yaml.safe_load(prev.read_text(encoding="utf-8")) or {}).get("slice_run") or {}).get("parts") or {})
            out = {k: v for k, v in out.items() if k not in want}
    # hr42（2026-09-24）：① 切片器子进程并行（--jobs，默认 SLICE_JOBS 或 4；每件自己的 --datadir，互不干扰）；
    #   ② 按内容哈希复用上次记录：键 = 本脚本 + l1_printable.py 源码、SETTINGS、切片器可执行文件（路径 + sha）、件号、源 STL sha、
    #      朝向/填充/周界/追加参数、禁撑区集合（no_support_zone_set 全量，全精度规范化）、采样步长、placed 名、Python/numpy/trimesh/shapely 版本
    #      → 与上次 slice_run.yaml 里该件记录的 slice_cache_key 相同就整条沿用，记录里加 slice_cached_from（= 当初真切的那次 run 标签）；
    #      --no-check-cache 或 DUCK_CHECK_CACHE=0 → 全部重切。
    #   前处理（摆朝向、导出）与后处理（解析 G-code、支撑落点）仍按件表顺序逐件做，记录顺序与字段与原来相同
    #   （另加 slice_cache_key、slice_run_tag = 本次 run 号，check_cache.run_tag()）。
    import os
    from concurrent.futures import ThreadPoolExecutor
    use_cache = not a.no_check_cache and os.environ.get("DUCK_CHECK_CACHE", "1") not in ("0", "off", "no", "false")
    jobs = a.jobs or (int(os.environ["SLICE_JOBS"]) if os.environ.get("SLICE_JOBS", "").isdigit() else 4)
    prev_parts = {}
    if (HERE / "slice_run.yaml").exists():
        prev_doc = yaml.safe_load((HERE / "slice_run.yaml").read_text(encoding="utf-8")) or {}
        prev_parts = dict(((prev_doc.get("slice_run") or {}).get("parts")) or {})
        prev_date = (prev_doc.get("slice_run") or {}).get("date")
    exe_sha = sha256(exe) if exe.is_file() else "?"
    sys.path.insert(0, str(ROOT / "tools" / "cad"))
    import check_cache as _CC                                        # run 号与检查原语缓存同一口径（go_chain 给整条链一个号）
    run_tag = _CC.run_tag()
    libv = _lib_versions()
    code_sha = [sha256(Path(__file__).resolve()), sha256(Path(L1.__file__).resolve())]
    jobs_todo, prep = [], {}
    reused = []
    for rec in parts:
        pid = rec["id"]
        if want and pid not in want:
            continue
        src = stl_index[pid]
        try:
            pc = part_cfg(P, pid)
        except KeyError as e:
            print(f"{e} —— 不给缺省朝向/填充率，退出", file=sys.stderr)
            return 4
        dl = down_local(pid, frames, pc["down_world"])
        infill = pc["fill_density_percent"]
        zone_set = L1.no_support_zone_set(P, feats_all, pid)
        key = hashlib.sha256(json.dumps(_canon(dict(v="hr42-slice-v2", code=code_sha, settings=SETTINGS, exe=str(exe), exe_sha=exe_sha,
                                                    pid=pid, src=sha256(src), dl=dl, infill=infill, pg=pc["perimeter_generator"],
                                                    extra=pc["slicer_args"], zones=zone_set, step=step, placed=placed_name(parts, pid),
                                                    libs=libv)),
                                        sort_keys=True, ensure_ascii=False).encode()).hexdigest()
        old = prev_parts.get(pid)
        if use_cache and old and old.get("slice_cache_key") == key:
            r = dict(old)
            r["slice_cached_from"] = old.get("slice_cached_from") or old.get("slice_run_tag") or f"slice_run {prev_date}"
            prep[pid] = dict(reuse=r); reused.append(pid)
            continue
        m = trimesh.load_mesh(str(src), process=False)
        M = trimesh.geometry.align_vectors(np.array(dl, dtype=float), [0, 0, -1])
        oriented = m.copy(); oriented.apply_transform(M)
        shift = np.array([100.0, 100.0, 0.0]) - np.array([*oriented.bounds.mean(0)[:2],
                                                          oriented.bounds[0, 2]])
        oriented.apply_translation(shift)
        opath = gdir / f"{pid}_oriented.stl"; oriented.export(opath)
        gpath = gdir / f"{pid}_analysis_only.gcode"
        extra = pc["slicer_args"]                                   # 逐件切片参数（printability.yaml:parts.<件>.slicer_args）
        cmd = [str(exe), *SETTINGS, "--fill-density", f"{infill}%", *extra,
               "--datadir", str(gdir / f"prusa_config_{pid}"),      # hr42：并行时每件自己的配置目录（原共用 prusa_config）
               "--save", str(logs / f"{pid}_profile.ini"),
               "--export-gcode", "--output", str(gpath), str(opath)]
        cmd[cmd.index("--perimeter-generator") + 1] = pc["perimeter_generator"]   # printability.yaml:parts.<件>.perimeter_generator（缺省 arachne）
        prep[pid] = dict(src=src, pc=pc, dl=dl, infill=infill, M=M, shift=shift, oriented=oriented, gpath=gpath, extra=extra,
                         cmd=cmd, zone_set=zone_set, key=key)
        jobs_todo.append(pid)

    def _slice(pid):
        t0 = time.perf_counter()
        p = subprocess.run(prep[pid]["cmd"], capture_output=True, text=True, timeout=900)
        return pid, p, round(time.perf_counter() - t0, 2)
    print(f"[hr42] 切片：重切 {len(jobs_todo)} 件（{jobs} 个切片器并行），沿用上次记录 {len(reused)} 件 {reused}", flush=True)
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
        done = {pid: (p, secs) for pid, p, secs in ex.map(_slice, jobs_todo)}
    for rec in parts:
        pid = rec["id"]
        if pid not in prep:
            continue
        if "reuse" in prep[pid]:
            out[pid] = prep[pid]["reuse"]
            print(pid, "沿用", out[pid].get("slice_cached_from"), flush=True)
            continue
        q = prep[pid]; src, pc, dl, infill, M, shift, oriented, gpath, extra, zone_set = (
            q["src"], q["pc"], q["dl"], q["infill"], q["M"], q["shift"], q["oriented"], q["gpath"], q["extra"], q["zone_set"])
        p, secs = done[pid]
        (logs / f"{pid}.log").write_text(p.stdout + p.stderr)
        warn = [l for l in (p.stdout + p.stderr).splitlines()
                if re.search(r"\b(?:warn(?:ing)?|error|repair(?:ed|ing)?|empty)\b", l, re.I)]
        rowvals = {"returncode": p.returncode, "seconds": secs, "warnings": warn}
        if p.returncode == 0 and gpath.exists():
            text = gpath.read_text(errors="replace")
            rowvals.update(parse_gcode(text, mesh=oriented))
            rowvals["gcode_sha256"] = sha256(gpath)
            rowvals["gcode_bytes"] = gpath.stat().st_size
            rowvals["support_landing"] = support_landing(text, M, shift, zone_set, step)
            rowvals["support_landing"]["zones_unlocated"] = [f for f, _ in zone_set["unlocated"]]
            rowvals["support_landing"]["zones_problems"] = list(zone_set["problems"])
        out[pid] = {
            "source_stl": str(src.relative_to(ROOT)),
            "source_sha256": sha256(src),
            "down_normal_export_local": dl,
            "down_normal_world": pc["down_world"],
            "placed_name": placed_name(parts, pid),
            "fill_density_percent": infill,
            "perimeter_generator": pc["perimeter_generator"],
            "slicer_argv_extra": extra,
            "orient_transform": {
                "align_vectors_M": [[float(round(v, 12)) for v in row] for row in np.asarray(M)],
                "shift": [float(round(v, 6)) for v in shift],
                "inverse": "p_export_local = inv(M)·(p_print − shift)；M = trimesh.geometry.align_vectors(down_normal_export_local, (0,0,-1))"},
            **rowvals,
            "slice_cache_key": q["key"],
            "slice_run_tag": run_tag,
        }
        sl = rowvals.get("support_landing") or {}
        print(pid, "rc", p.returncode, "layers", rowvals.get("layers"),
              "support_mm3", rowvals.get("support_volume_mm3"),
              "single_bead", rowvals.get("layers_single_bead"),
              "landing forbid/removable", sl.get("hits_forbid_samples"), "/", sl.get("hits_removable_samples"),
              "zones", sl.get("zones_n"), "extra", extra,
              f"{secs}s", "warn", warn[:1], flush=True)

    doc = {
        "slice_run": {
            "slicer": f"PrusaSlicer 2.9.6（官方 macOS 版；可执行文件 {exe}；"
                      "dmg sha256 94fd7b8a9f87c9631e1c71739b15b184fc5f4c0ceabd69072f1c78f229a4fe40，"
                      "github.com/prusa3d/PrusaSlicer release version_2.9.6，09-13 装到 /Applications）",
            "date": time.strftime("%Y-%m-%d"),
            "runner": "tools/gate/slicing/slice_l1.py",
            "input": "cad/duck_s288/*.stl（导出件自身坐标系，元规则 1）",
            "gcode_kept": False,
            "gcode_note": "G-code 共约 75 MB，不入库；重跑 slice_l1.py 可复现（sha256 已记录）",
            "settings_argv": SETTINGS,
            "per_part_argv_source": "printability.yaml:parts.<件>.slicer_args → slice_run.parts.<件>.slicer_argv_extra",
            "support_landing_note": ("slice_run.parts.<件>.support_landing：G-code 支撑挤出段按 criteria.support_landing_sample_step_mm 采样、"
                                     "经 orient_transform 逆变换回 export_local，对 l1_printable.no_support_zone_set 建出的禁撑区逐区数点；"
                                     "zones_sha256 = 当时的禁撑区声明 hash，层核对不上判 unknown"),
            "parts": out,
        }
    }
    (HERE / "slice_run.yaml").write_text(
        yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, width=200), encoding="utf-8")
    print("→", HERE / "slice_run.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
