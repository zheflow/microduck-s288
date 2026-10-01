#!/usr/bin/env python3
"""逐关节静态受力：算出每个舵机输出轴在**法兰面**上承受的弯矩，用来判断哪些关节需要副支撑。

为什么看弯矩而不是看扭矩：舵机的额定扭矩说的是绕轴转的能力，和它能扛多大的**侧向弯曲**没关系。
S288 是全塑料件（齿轮、壳、法兰都是塑料自攻），输出轴颈本身就是个塑料轴颈，
单侧悬臂挂着一条腿的时候，弯矩全压在这一个轴颈上；两端都接上（法兰 + 背面惰轮）或者法兰侧加一颗轴承，
弯矩可由分隔支点的径向反力分担；输出轴仍承受径向力，具体分载取决于刚度、间隙和装配。
本工具不能推导轴承/惰轮额定容量，也不能把增加支点等同于承载已经合格。

两个工况（都是静态、零位姿，只用来排序哪个关节最危险，不是动力学仿真）：
  · 悬垂远端：只有该关节**远端子树**自身的重量挂着（抬腿摆动相、或者头/脖子平时的状态）
  · 单腿支撑：整机重量全部通过一条腿的脚底接触点传上来（走路时最重的那一瞬间）。只对腿上的关节有意义。

**这里算不出"安不安全"**：S288 输出轴承的径向额定手册没给（tolerances.yaml:load.servo_output_bearing.rated_radial_N = null），
所以没有额定值可比。本表只能做**关节之间的横向排序**，以及和该关节自身驱动扭矩比大小。
真正的判据是实测：bench.yaml:BN08（7 个部位的间隙/回差增长）。

    ./.venv/bin/python tools/cad/loadcheck.py
"""
import os, sys, xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import duckstructure as D
from duckstructure import s288
from duckstructure.s288 import S

MJCF = "sim/duck_s288/robot_walk_s288.xml"
G = 9.80665

def load_inertials():
    """从 S288 版 MJCF 读每个 body 的质量与质心（本体系，mm）+ 父子关系与零位姿世界变换。"""
    root = ET.parse(MJCF).getroot()
    from trimesh.transformations import quaternion_matrix

    def T(pos, quat):
        p = np.array([float(x) for x in (pos or "0 0 0").split()]) * 1000.0
        M = quaternion_matrix([float(x) for x in (quat or "1 0 0 0").split()])
        M[:3, 3] = p
        return M

    bodies, order = {}, []

    def walk(b, parent, Tw):
        n = b.get("name")
        Twb = Tw @ T(b.get("pos"), b.get("quat"))
        ine = b.find("inertial")
        m_g = float(ine.get("mass")) * 1000.0 if ine is not None else 0.0
        com = np.array([float(x) for x in ine.get("pos").split()]) * 1000.0 if ine is not None else np.zeros(3)
        bodies[n] = dict(parent=parent, Tw=Twb, mass_g=m_g, com_world=(Twb @ np.append(com, 1.0))[:3], kids=[])
        order.append(n)
        if parent is not None:
            bodies[parent]["kids"].append(n)
        for c in b.findall("body"):
            walk(c, n, Twb)

    walk(root.find("worldbody").find("body"), None, np.eye(4))
    return bodies, order

def subtree(bodies, n):
    out = [n]
    for k in bodies[n]["kids"]:
        out += subtree(bodies, k)
    return out

def lump(bodies, names):
    m = sum(bodies[k]["mass_g"] for k in names)
    if m <= 0:
        return 0.0, np.zeros(3)
    c = sum(bodies[k]["mass_g"] * bodies[k]["com_world"] for k in names) / m
    return m, c

def split(M, axis):
    """把力矩分成【绕关节轴】（= 舵机要扛的驱动扭矩）和【垂直于关节轴】（= 压在轴颈上的弯矩）两部分。"""
    a = axis / np.linalg.norm(axis)
    drive = float(M @ a)
    return abs(drive), float(np.linalg.norm(M - drive * a))

if __name__ == "__main__":
    bodies, order = load_inertials()
    total_g = sum(b["mass_g"] for b in bodies.values())

    # 脚底接触面：取左鞋底 L06 底面（zmin 往上 0.5 那一层）的所有点。
    # 名义接触点用这一层的形心；单腿支撑时压心会跑到脚掌边缘，所以另取这一层的**凸包角点**做最坏情况。
    import trimesh
    sole = trimesh.load("cad/duck_s288/placed/sole.stl", process=True)
    v = sole.vertices
    bot = v[v[:, 2] <= v[:, 2].min() + 0.5]
    contact = bot.mean(axis=0)
    from scipy.spatial import ConvexHull
    hull2d = bot[ConvexHull(bot[:, :2]).vertices][:, :2]
    corners = [np.array([p[0], p[1], float(bot[:, 2].min())]) for p in hull2d]
    leg_chain = set(subtree(bodies, "yaw2roll"))       # 左腿这条链上的关节才适用单腿支撑

    print(f"整机 {total_g:.1f} g（MJCF inertial 合计，含电池等已录入的元件）")
    print(f"左鞋底底面 z={bot[:, 2].min():.2f}，形心 {np.round(contact, 1).tolist()}，边缘取样 {len(corners)} 点")
    print("弯矩 = 对**法兰面中心**取矩后垂直于关节轴的分量；驱动矩 = 沿关节轴的分量。单位 N·mm。")
    print("单腿支撑是完整自由体：远端子树受 ①脚底地面反力（整机重量，向上）②自身重量（向下），两者一起取矩。\n")
    hdr = f"{'关节':18s}{'远端质量g':>9s}  {'工况':10s}{'弯矩':>8s}{'边缘最坏':>9s}{'驱动矩':>8s}"
    print(hdr)
    print("-" * len(hdr))

    for body in D.ORDER:
        for i, sv in enumerate(D.B[body]["servos"]):
            if sv["drives"] is None:
                continue
            tag = sv["drives"].replace(":self", "")
            if tag not in bodies:
                continue
            R = D.sfw(body, i)
            P = D.pt(R, s288.x_flange_face())            # 法兰面中心（力矩取矩点）
            axis = bodies[tag]["Tw"][:3, 2]              # 关节轴 = 子 body 的 z
            m_g, com = lump(bodies, subtree(bodies, tag))
            Wd = np.array([0.0, 0.0, -m_g / 1000.0 * G])           # 远端子树自重
            drive, bend = split(np.cross(com - P, Wd), axis)
            print(f"{tag:18s}{m_g:9.1f}  {'悬垂远端':10s}{bend:8.1f}{'':>9s}{drive:8.1f}")
            if tag in leg_chain:
                N = np.array([0.0, 0.0, total_g / 1000.0 * G])     # 地面反力
                def at(c):
                    return split(np.cross(c - P, N) + np.cross(com - P, Wd), axis)
                d0, b0 = at(contact)
                worst = max(at(c)[1] for c in corners)
                print(f"{'':18s}{total_g:9.1f}  {'单腿支撑':10s}{b0:8.1f}{worst:9.1f}{d0:8.1f}")
    print("\n注：单腿支撑把整机重量整块压在一只脚上，是**静态上界**（双脚站立时每边约一半；走路落地冲击会更大）。")
    print("    S288 输出轴承径向额定手册未给 → 本表不能判定'超载'，只能做关节间排序 + 与该关节驱动扭矩比大小。")
