"""生成 robot_walk_s288_allcol_play.xml：全碰撞变体的物理（body/joint/inertial/碰撞几何逐字节同 robot_walk_s288_allcol.xml）
+ 回放变体的整件涂装视觉（vis_*.stl、材质、相机眼睛，取自 robot_walk_s288_play.xml）。
只供起立/坐站/翻身/捡物/踢球的 play/录像用，Gate 不读，训练不用。

背景（09-19）：allcol 变体里壳/头壳/躯干是 class=collision（group 3），mjlab 渲染默认不画 group 3，录像里鸭子看起来
"没头没壳"；物理其实一直有（质量 762.7 g 三份 XML 相同，壳的凸包在碰撞集里）。

用法: ./.venv/bin/python -B tools/sim/make_allcol_play.py   （在 make_mjcf.py 之后跑）
"""
import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path

SIM = Path(__file__).resolve().parents[2] / "sim" / "duck_s288"
allcol = ET.parse(SIM / "robot_walk_s288_allcol.xml").getroot()
play = ET.parse(SIM / "robot_walk_s288_play.xml").getroot()

root = copy.deepcopy(allcol)
# 1. 去掉 allcol 里留作视觉的凸包碎片（class=visual 的 col_*），换成整件网格
n_rm = 0
for b in root.iter("body"):
    for gm in list(b.findall("geom")):
        if gm.get("class") == "visual" and (gm.get("name") or "").startswith("col_"):
            b.remove(gm); n_rm += 1
# 2. 材质 + vis_* 网格资产
asset = root.find("asset"); passet = play.find("asset")
have = {m.get("name") for m in asset.findall("material")}
for m in passet.findall("material"):
    if m.get("name") not in have:
        asset.append(copy.deepcopy(m))
have_mesh = {m.get("name") for m in asset.findall("mesh")}
n_mesh = 0
for m in passet.findall("mesh"):
    if m.get("name", "").startswith("vis_") and m.get("name") not in have_mesh:
        asset.append(copy.deepcopy(m)); n_mesh += 1
# 3. 每个 body 的 vis_* 几何（含眼睛）按 body 名复制过去
bodies = {b.get("name"): b for b in root.iter("body")}
n_vis = 0
for pb in play.iter("body"):
    tb = bodies.get(pb.get("name"))
    if tb is None:
        raise SystemExit(f"play 里的 body {pb.get('name')} 在 allcol 里找不到")
    for gm in pb.findall("geom"):
        if (gm.get("name") or "").startswith("vis_"):
            tb.append(copy.deepcopy(gm)); n_vis += 1
# 4. 文件头注释
for c in [c for c in root if c.tag is ET.Comment]:
    root.remove(c)
root.insert(0, ET.Comment(" allcol_play 变体：物理 = robot_walk_s288_allcol.xml（逐字节同），视觉 = robot_walk_s288_play.xml 的涂装网格；"
                          "由 tools/sim/make_allcol_play.py 生成，只供 play/录像 "))
ET.indent(root, space="  ")
out = SIM / "robot_walk_s288_allcol_play.xml"
out.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="unicode"), encoding="utf-8")
print(f"[allcol_play] 去掉凸包视觉 {n_rm}，加 vis 网格 {n_mesh}、vis 几何 {n_vis} → {out}")

# 5. 物理一致性自检：编译后 body/joint/质量/惯量/碰撞几何完全相同
import mujoco, numpy as np
ma = mujoco.MjModel.from_xml_path(str(SIM / "robot_walk_s288_allcol.xml")); mp = mujoco.MjModel.from_xml_path(str(out))
def colset(m):
    return sorted((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i), int(m.geom_contype[i]), int(m.geom_conaffinity[i]), int(m.geom_condim[i]),
                   mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[i]))
                  for i in range(m.ngeom) if m.geom_contype[i] or m.geom_conaffinity[i])
assert ma.nbody == mp.nbody and ma.njnt == mp.njnt and ma.nu == mp.nu
assert np.allclose(ma.body_mass, mp.body_mass) and np.allclose(ma.body_inertia, mp.body_inertia) and np.allclose(ma.body_ipos, mp.body_ipos)
assert np.allclose(ma.jnt_range, mp.jnt_range) and colset(ma) == colset(mp)
print(f"[allcol_play] 自检通过：body {mp.nbody} joint {mp.njnt} 质量 {mp.body_subtreemass[0]*1000:.1f} g 碰撞几何 {len(colset(mp))}（与 allcol 相同）")
