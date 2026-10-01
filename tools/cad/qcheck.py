#!/usr/bin/env python3
"""快速子集检查：只重建指定的件 + 相关邻件/舵机，跑零位姿干涉 + 关节扫掠。
整机 `python -m duckstructure.build` 要跑好几分钟；改一个件时用这个先过一遍。

    python tools/cad/qcheck.py head     # N01/N02/N03/H01 + 头壳 + 躯干 + 躯干壳
    python tools/cad/qcheck.py trunk    # T01/B01/T02/T03 + 两条腿 + 电池
    python tools/cad/qcheck.py all

判据与 duckstructure/build.py 一致：干涉阈值 0.05 mm³；布尔失败会抛出异常并中止。默认与整机一样查 MJCF 全行程。
"""
import math, numpy as np, trimesh
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "cad"))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import duckstructure as D
from duckstructure import s288
from duckstructure.s288 import S
from trimesh.transformations import rotation_matrix as rot

WHICH = sys.argv[1] if len(sys.argv) > 1 else "all"
parts = []
def add(body, name, m): parts.append((body, name, m))

if WHICH in ("head", "all"):
    add("neck", "neck", D.build_neck())
    add("neck_pitch", "neck_pitch", D.build_neck_pitch())
    add("yaw_roll_motion", "yrm", D.build_yrm())
    add("yaw_roll_motion", "head_journal", D.build_head_journal())
    add("jaw_soft", "head_bracket", D.build_head_bracket())
    add("jaw_soft", "head_clamp", D.build_head_clamp())
if WHICH in ("trunk", "all"):
    tr = D.build_trunk(); add("trunk_base", "trunk", tr)
    add("trunk_base", "battery_door", D.build_battery_door())
    add("trunk_base", "zz_battery", D.battery_box())     # hr48：同 build.py
    add("trunk_base", "shell_L", D.build_trunk_shell(1, tr))
    add("trunk_base", "shell_R", D.build_trunk_shell(-1, tr))
    for b, n, m in [("yaw2roll","yaw2roll",D.build_yaw2roll()), ("hip_l","hip",D.build_hip()),
                    ("upper_leg_left","upper_leg",D.build_upper_leg()), ("leg","lower_leg",D.build_lower_leg())]:
        add(b, n, m); add(D.MIRROR[b], n + "_R", D.mirror_y(m))
    af, sole = D.build_ankle_foot(); add("ankle_left","ankle_foot",af); add("ankle_right","ankle_foot_R",D.mirror_y(af))
    rear = D.build_ankle_rear_arm()
    for name, mesh in [('sole',sole),('ankle_rear_arm',rear)]:
        add('ankle_left',name,mesh); add('ankle_right',name+'_R',D.mirror_y(mesh))
if WHICH == "head":
    tr = D.build_trunk(); add("trunk_base", "trunk", tr)
    add("trunk_base", "shell_L", D.build_trunk_shell(1, tr)); add("trunk_base", "shell_R", D.build_trunk_shell(-1, tr))

add('jaw_soft','head_bottom_shell',D.build_head_bottom_shell())
for p in D.B["jaw_soft"]["parts"]:
    if p["mesh"] in ("top_head_shell", "jaw", "face_part"):
        m = D.K.mesh(p["mesh"]); m.apply_transform(D.TW("jaw_soft") @ p["T"]); add("jaw_soft", "orig_" + p["mesh"], m)

servos = []
for n in D.ORDER:
    for i, s in enumerate(D.B[n]["servos"]):
        if s["drives"] is None: continue
        servos.append((n, f"servo_{n}_{s['drives'].replace(':self','')}", s288.placed(s288.servo_mesh(), D.sfw(n, i))))
from assembly_audit import bearing_specs
bearings=[]
for spec in bearing_specs(D):
    raw=spec['shape'].to_mesh64()
    mesh=trimesh.Trimesh(vertices=np.asarray(raw.vert_properties)[:,:3],faces=np.asarray(raw.tri_verts),process=False)
    bearings.append((spec['mount'],'bearing_'+spec['name'],mesh))
allm = parts + servos + bearings
for b, n, m in parts:
    nt = D.keep_main(m, n); c = nt.split(only_watertight=False)
    print(f"{n:20s} {str(np.round(m.bounds[1]-m.bounds[0],1)):22s} {m.volume/1000:6.2f}cm3 散件{len(c)}")

result = D.run_checks(allm)
sys.exit(0 if result["sampled_collision_free"] else 2)
