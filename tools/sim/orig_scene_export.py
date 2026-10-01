#!/usr/bin/env python3
"""把**原版 Microduck** 的全部网格按零位姿摆到世界系，导出成 L6 场景能读的 placed 目录 —— 用来和我们的件在同一组姿态下对比碰撞。

    ./.venv/bin/python tools/sim/orig_scene_export.py
    → tools/gate/out/orig_placed_2026-09-16/<body>__<mesh>.stl + body_for_part.json

来源：upstream/microduck_rl/.../robot_allcollisions.xml（全部 mesh geom，视觉+碰撞去重，含 XL330/轴承/电子件）。
零位姿 body 世界位置与 duckstructure.lib.B[*]["T_world"] 逐字一致（已核：trunk_base/hip_l/leg/ankle_left/neck/jaw_soft）。
"""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np, mujoco, trimesh

ROOT = Path(__file__).resolve().parents[2]
XML = ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_allcollisions.xml"
OUT = ROOT / "tools/gate/out/orig_placed_2026-09-16"


def main():
    m = mujoco.MjModel.from_xml_path(str(XML)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    assets = XML.parent / "assets"
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob("*.stl"):
        f.unlink()
    bfp, seen, n = {}, set(), 0
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        mesh_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
        key = (body, mesh_name, tuple(np.round(d.geom_xpos[g] * 1000, 2)))     # 同 body 同网格同位置 = 视觉/碰撞重复；不同位置 = 第二颗舵机
        if key in seen:
            continue
        seen.add(key)
        mid = m.geom_dataid[g]
        v0, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        f0, nf = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        verts = m.mesh_vert[v0:v0 + nv].copy(); faces = m.mesh_face[f0:f0 + nf].copy()
        mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        T = np.eye(4); T[:3, :3] = d.geom_xmat[g].reshape(3, 3); T[:3, 3] = d.geom_xpos[g]
        mesh.apply_transform(T); mesh.apply_scale(1000.0)             # m → mm（我们的 placed 是 mm）
        stem = f"{body}__{mesh_name}"
        k = 2
        while (OUT / f"{stem}.stl").exists():
            stem = f"{body}__{mesh_name}_{k}"; k += 1
        mesh.export(str(OUT / f"{stem}.stl")); bfp[stem] = body; n += 1
    (OUT / "body_for_part.json").write_text(json.dumps({"body_for_part": bfp, "source": str(XML.relative_to(ROOT))}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{n} 个原版网格 → {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
