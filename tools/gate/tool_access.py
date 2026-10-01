"""L4 起子路径：真坐面、批杆包络、拧紧时的工位。"""
import re
import numpy as np
import trimesh
from core import num, PASS, FAIL, BLOCK


def tool_state(asm, ref):
    from layers.l4_assembly import motion_coverage
    if isinstance(ref, str) and ref.startswith("motion:"):
        _, step, mid = ref.split(":", 2)
        st = next(s for s in asm["assembly_order"] if s["step"] == int(step))
        motion_coverage(st)
        return st["required_motion_groups"][mid]
    if isinstance(ref, str) and ref.startswith("tool:"):
        return asm["tool_states"][ref[5:]]
    raise ValueError("tool_access.state_ref 缺失；不能默认把完整机器人当成拧紧时的工位")


def seat_samples(mesh, point, axis, hole_d, head_d):
    """头足印内側有料、外側为空。0.001 mm 是离面数值探针偏移，不是配合公差。"""
    from layers.l5_screwhead import _ring, _ring_radii
    if not (0 < hole_d < head_d):
        raise ValueError("螺丝头必须比过孔大，足印才存在")
    pts = np.vstack([_ring(np.asarray(point), axis, r) for r in _ring_radii(hole_d / 2, head_d / 2)])
    inward = mesh.contains(pts - np.asarray(axis) * 0.001)
    outward = mesh.contains(pts + np.asarray(axis) * 0.001)
    return dict(ok=bool(inward.all() and not outward.any()), evidence_n=2 * len(pts),
                material_inward=int(inward.sum()), void_outward=int((~outward).sum()), n=len(pts))


# hr41 落盘 2026-09-25（主设计 Lane C）：沉头锥面坐面（hr41 §8「L4 tool_access 平面环带测试不适合锥面坐面」；首例 F34 90° 沉头机牙）。
#   坐面声明 seats[].cone = {included_angle_deg: {v,src}, top_d_mm: {v,src}[, head_d_mm: {v,src}]} 时走本函数，否则仍走上面的平面 seat_samples（逐字节不变）。
def cone_seat_samples(mesh, point, axis, hole_d, head_d, top_d, included_deg):
    """锥面坐面：point = 孔轴上锥口直径 = top_d 的那一截面（锥口顶面），axis = 头侧（向外）。
    头足印环带与平面坐面同一套 _ring/_ring_radii（半径 hole_d/2 .. min(head_d, top_d)/2，等面积 3 圈 × 24 向），但每圈落在**锥面**上：
      α = included_deg/2；半径 r 处锥面在 point 内侧 (top_d/2 − r)/tanα；锥面法向（指向锥口空腔）n = axis·sinα − e·cosα（e = 径向单位向量）。
    判据与平面坐面同义：锥面料侧 p − 0.001·n 有料、空腔侧 p + 0.001·n 为空（0.001 mm 是离面数值探针偏移，不是配合公差）。"""
    import math
    from layers.l5_screwhead import _ring, _ring_radii
    if not (0 < included_deg < 180):
        raise ValueError(f"锥面夹角 {included_deg} 必须在 (0, 180)")
    outer = min(head_d, top_d)
    if not (0 < hole_d < outer):
        raise ValueError("螺丝头/锥口必须比过孔大，锥面足印才存在")
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    al = math.radians(included_deg) / 2.0
    pts, nrm = [], []
    for r in _ring_radii(hole_d / 2, outer / 2):
        c = np.asarray(point, float) - a * ((top_d / 2 - r) / math.tan(al))
        ring = _ring(c, a, r)
        e = (ring - c) / r
        pts.append(ring)
        nrm.append(a * math.sin(al) - e * math.cos(al))
    pts, nrm = np.vstack(pts), np.vstack(nrm)
    inward = mesh.contains(pts - nrm * 0.001)
    outward = mesh.contains(pts + nrm * 0.001)
    return dict(ok=bool(inward.all() and not outward.any()), evidence_n=2 * len(pts), shape=f"cone{included_deg:g}",
                material_inward=int(inward.sum()), void_outward=int((~outward).sum()), n=len(pts))


def _cone_decl(cone, head_d_default):
    """seats[].cone → (included_deg, top_d, head_d)。夹角/锥口径必须 {v>0, src}；head_d_mm 可缺（缺则用 frozen 同规格头径）。"""
    out = []
    for key in ("included_angle_deg", "top_d_mm"):
        node = cone.get(key)
        v, _ = num(node)
        if v is None or not np.isfinite(v) or v <= 0 or not (isinstance(node, dict) and node.get("src")):
            raise ValueError(f"锥面坐面 cone.{key} 缺值/非正/缺 src")
        out.append(float(v))
    hd = head_d_default
    if cone.get("head_d_mm") is not None:
        v, _ = num(cone.get("head_d_mm"))
        if v is None or not np.isfinite(v) or v <= 0 or not (isinstance(cone.get("head_d_mm"), dict) and cone["head_d_mm"].get("src")):
            raise ValueError("锥面坐面 cone.head_d_mm 缺值/非正/缺 src")
        hd = float(v)
    return out[0], out[1], hd


def run_tool_access(res, ctx, geo, names, asm, tol):
    from layers.l4_assembly import _world_of_hole, _reach_to_outside, judge_tool_channel
    from layers.l5_screwhead import _head_geom, _declared_axis
    frames = ((ctx.data.get("features") or {}).get("frames") or {}).get("per_part") or {}
    features = {f["id"]: f for f in (ctx.data.get("features") or {}).get("features", [])}
    for fa in (ctx.data.get("fasteners") or {}).get("fasteners", []):
        gid = fa["id"]
        if fa.get("joint_type") in {"none_zip_tie", "pla_snap", "none_no_receiver"}:
            continue  # 非螺丝的功能与禁用状态由 L5 判
        provenance = f"fasteners.yaml:{gid}.tool_access/tool_envelope | assembly.yaml"
        try:
            ops = fa.get("tool_access")
            if not isinstance(ops, list) or not ops:
                raise ValueError("未声明 tool_access 真坐面/头侧方向/拧紧工位；旧 feature_hole_map 是孔或切刀中点，不是坐面")
            bit_d, _ = num((fa.get("tool_envelope") or {}).get("bit_d_mm"))
            if bit_d is None or not np.isfinite(bit_d) or bit_d <= 0:
                raise ValueError("缺本组 bit_d_mm；沉孔/内腔 channel_d_mm 不能代替批杆")
            spec = re.search(r"M(\d+(?:\.\d+)?)", str(fa.get("spec", "")))
            if spec is None:
                raise ValueError("缺螺丝主径规格，无法验证承压足印")
            head_d = _head_geom(ctx.data.get("frozen"), float(spec.group(1)))[0]
            if head_d is None:
                raise ValueError("缺本规格螺丝头尺寸")
            if type(fa.get("qty")) is not int or fa["qty"] <= 0:
                raise ValueError("螺丝 BOM qty 必须为正整数")
            locators = fa.get("head_locator_map_indices")
            if (not isinstance(locators, list) or not locators or len(set(locators)) != len(locators)
                    or any(type(i) is not int or i < 0 for i in locators)):
                raise ValueError("缺独立 head_locator_map_indices；必须先声明哪些映射项是头侧孔全集")
            expected, physical_axes = set(), {}                   # hr41 落盘 2026-09-25（Lane C）：孔轴线 → 已登记孔的轴向坐标
            for mi in locators:
                entry = fa["feature_hole_map"][mi]
                hs = entry.get("holes")
                feature = features[entry["feature_id"]]
                part = feature["part"]
                if not isinstance(hs, list) or not hs or len(hs) != entry.get("n"):
                    raise ValueError("头侧孔映射的 n 与坐标数不符")
                coords = np.asarray(hs, float)
                if (coords.shape != (len(hs), 3) or not np.isfinite(coords).all()
                        or len(np.unique(coords, axis=0)) != len(hs)):
                    raise ValueError("头侧孔映射必须是有限三维坐标且逐孔唯一")
                fg = feature.get("geom") or {}
                if fg.get("instances"):
                    fg = {**fg, **fg["instances"][entry["instance"]]}
                axis = _declared_axis(fg.get("axis"))
                if axis is None:
                    raise ValueError("头侧孔轴未声明")
                axis = axis if axis[np.flatnonzero(axis)[0]] > 0 else -axis
                # hr41 落盘 2026-09-25（主设计 Lane C）：同轴异位的两个孔（F34：J01 两毂各一个沉头孔，同一轴线、轴向相距 106.8）不是"同一孔凑两颗"。
                #   只有特征声明了孔深 depth_mm（>0）时才按轴向位置区分：同一实体同一孔轴、轴向相距 ≤ 孔深 → 仍判同一孔（原规则）；没声明孔深 → 原规则不变。
                dep, _ = num(fg.get("depth_mm"))
                dep = float(dep) if dep is not None and np.isfinite(dep) and dep > 0 else None
                for host in names.part_stems(part):
                    for hi, anchor in enumerate(coords):
                        line = (host, *np.round(np.r_[axis, anchor - axis * (anchor @ axis)], 8))
                        t_ax = float(anchor @ axis)
                        prev = physical_axes.get(line, [])
                        if prev and (dep is None or any(abs(t_ax - t0) <= dep for t0 in prev)):
                            raise ValueError("同一实体同一孔轴跨映射重复，不能凑成两颗螺丝"
                                             + ("" if dep is None else f"（轴向相距 ≤ 孔深 {dep:g}）"))
                        physical_axes.setdefault(line, []).append(t_ax)
                        expected.add((host, mi, hi))
            if len(expected) != fa.get("qty"):
                raise ValueError(f"头侧孔义务 {len(expected)} ≠ BOM 螺丝数 {fa.get('qty')}")
            seats, seen, seen_holes = [], set(), set()
            for op in ops:
                state = tool_state(asm, op.get("state_ref"))
                present = state.get("present")
                if (not state.get("workspace") or not isinstance(present, list) or not present
                        or len(set(present)) != len(present) or not set(present) <= set(geo.index)):
                    raise ValueError("工位在场全集缺失/重复/含未导出对象")
                for seat in op.get("seats") or []:
                    feat = features[seat["feature_id"]]
                    part, host = feat["part"], seat["instance"]
                    stems = names.part_stems(part)
                    if host not in stems or host not in present:
                        raise ValueError("坐面承件未在此工位，或不是该打印件的实例")
                    if any(type(seat.get(k)) is not int or seat[k] < 0 for k in ("map_index", "hole_index")):
                        raise ValueError("孔索引必须为非负整数")
                    identity = (host, seat["map_index"], seat["hole_index"])
                    if identity not in expected or identity in seen_holes:
                        raise ValueError("头侧孔身份重复/不属于本组义务，不能用同一孔的两个表面凑数")
                    seen_holes.add(identity)
                    pt, ax = np.asarray(seat["point_export_local"], float), np.asarray(seat["outward_export_local"], float)
                    if pt.shape != (3,) or ax.shape != (3,) or not np.isfinite([pt, ax]).all() or np.linalg.norm(ax) <= 0:
                        raise ValueError("坐面点/头侧方向必须是有限三维向量")
                    ax /= np.linalg.norm(ax)
                    entry = fa["feature_hole_map"][seat["map_index"]]
                    if entry["feature_id"] != seat["feature_id"]:
                        raise ValueError("坐面未绑定本组螺丝的特征孔")
                    anchor = np.asarray(entry["holes"][seat["hole_index"]], float)
                    delta = pt - anchor
                    if np.linalg.norm(delta - ax * (delta @ ax)) > 1e-6:
                        raise ValueError("坐面不在所绑定孔轴上；孔/坐面数据不得横向错位")
                    fg = feat.get("geom") or {}
                    if fg.get("instances"):
                        fg = {**fg, **fg["instances"][entry["instance"]]}
                    declared = _declared_axis(fg.get("axis"))
                    if declared is None or abs(float(declared @ ax)) < 1 - 1e-9:
                        raise ValueError("头侧轴向与绑定特征轴线不符/未声明")
                    world = _world_of_hole(frames, part, pt)
                    ax = np.asarray(frames[part]["world_to_export_local_R"], float).T @ ax
                    ax /= np.linalg.norm(ax)
                    if host != stems[0]:  # build.py 的 mirror_y 导出约定
                        world[1] *= -1; ax[1] *= -1
                    key = (host, *np.round(world, 8))
                    if key in seen:
                        raise ValueError("同一螺丝坐面被重复计数，不能補齐另一颗")
                    seen.add(key)
                    hole_d, _ = num((feat.get("geom") or {}).get("nominal_d_mm"))
                    if hole_d is None:
                        raise ValueError("坐面所关联的过孔缺孔径")
                    if seat.get("cone") is not None:          # hr41 落盘 2026-09-25（Lane C）：沉头锥面坐面
                        if not isinstance(seat["cone"], dict):
                            raise ValueError("seats[].cone 必须是 {included_angle_deg, top_d_mm[, head_d_mm]}")
                        ang, top_d, hd_c = _cone_decl(seat["cone"], head_d)
                        check = cone_seat_samples(geo.mesh(host), world, ax, hole_d, hd_c, top_d, ang)
                    else:
                        check = seat_samples(geo.mesh(host), world, ax, hole_d, head_d)
                    seats.append(dict(host=host, point=world, axis=ax, present=present, seat=check))
            if seen_holes != expected or len(seats) != fa.get("qty") or not seats:
                raise ValueError(f"坐面覆盖 {len(seats)} ≠ BOM 螺丝数 {fa.get('qty')}")
        except (KeyError, IndexError, ValueError, TypeError, StopIteration) as e:
            res.unknown(gid, "tool_reference", str(e), provenance=provenance)
            continue
        valid = all(s["seat"]["ok"] for s in seats)
        cones = sorted({s["seat"]["shape"] for s in seats if s["seat"].get("shape")})
        res.add(subject=gid, check="tool_origin_on_seat", state=PASS if valid else FAIL, severity=BLOCK,
                measured=[s["seat"] for s in seats], evidence_n=sum(s["seat"]["evidence_n"] for s in seats),
                criterion="每颗显式头侧坐面：头足印向内有料、向外为空；不得从孔中点或挡墙外起扫"
                          + (f"（锥面坐面 {cones}：足印环带落在锥面上，沿锥面法向 ±0.001 取料/空，tool_access.cone_seat_samples）"
                             if cones else ""),
                provenance=provenance)
        if not valid:
            continue
        rows, visible, rays = [], 0, 0
        for s in seats:
            bounds = [geo.bounds(n) for n in s["present"]]
            wb = (np.min([b[0] for b in bounds], axis=0), np.max([b[1] for b in bounds], axis=0))
            # 坐面若已在工位外表面，仍向外查一个批杆直径，避免零长度证据。
            reach = max(_reach_to_outside(wb, s["point"], s["axis"]), float(bit_d))
            row = judge_tool_channel(geo, s["point"], s["axis"], bit_d, reach, tol, s["present"])
            row.update(reach=reach, host=s["host"], point=s["point"].tolist())
            rows.append(row)
            mesh = trimesh.util.concatenate([geo.mesh(n) for n in s["present"]])
            hits = mesh.ray.intersects_location(np.array([s["point"] + s["axis"] * 0.001]), np.array([s["axis"]]))[0]
            distances = (hits - s["point"]) @ s["axis"] if len(hits) else np.array([])
            visible += int(not np.any((distances > 0.001) & (distances <= reach)))
            rays += 1
        peak = max(r["peak"] for r in rows)
        res.add(subject=gid, check="tool_path_to_outside", state=PASS if peak <= tol else FAIL, severity=BLOCK,
                measured=peak, evidence_n=sum(r["evidence_n"] for r in rows),
                criterion=f"逐颗 Ø{bit_d} 批杆从真实坐面到拧紧工位外，全程交集 ≤ {tol} mm³",
                detail=str(rows) + "；只证明所声明装配工位的拧紧路径，全机维修拆卸另判。", provenance=provenance)
        reach = max(r["reach"] for r in rows)
        available, _ = num((fa.get("tool_envelope") or {}).get("available_shaft_len_mm"))
        if available is None or not np.isfinite(available) or available <= 0:
            res.unknown(gid, "tool_reach_declared",
                        f"需要杆长 ≥ {reach:.6f} mm；缺 available_shaft_len_mm。旧 shaft_len_mm 是几何需求下界，不能当实物可用杆长。",
                        provenance=provenance)
        else:
            res.add(subject=gid, check="tool_reach_declared", state=PASS if available >= reach else FAIL, severity=BLOCK,
                    measured={"required_mm": reach, "available_mm": available}, evidence_n=len(rows),
                    criterion="本组声明可用批杆长度 ≥ 逐颗坐面到工位外所需长度", provenance=provenance)
        res.add(subject=gid, check="screw_head_visible", state=PASS if visible == len(seats) else FAIL, severity=BLOCK,
                measured=f"{visible}/{len(seats)}", evidence_n=rays,
                criterion="每颗从真实坐面沿头侧方向到工位外的轴向射线均畅通", provenance=provenance)
