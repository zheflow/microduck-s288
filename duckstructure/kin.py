"""从 microduck_rl 的 robot_walk.xml 抽取整鸭运动学（关节轴、舵机位姿、轴承、原件），单位 mm。
所有 CAD 的关节轴线都以这里为准 —— 这样训练好的策略/仿真骨架不用动。"""
import os, xml.etree.ElementTree as ET, numpy as np, trimesh
from trimesh.transformations import quaternion_matrix
HERE = os.path.dirname(os.path.abspath(__file__))
MD = os.path.normpath(os.path.join(HERE, "..", "upstream", "microduck_rl", "src", "mjlab_microduck", "robot", "microduck"))
ASSETS = os.path.join(MD, "assets")

def _T(pos, quat):
    p = np.array([float(x) for x in pos.split()]) * 1000.0
    M = quaternion_matrix([float(x) for x in quat.split()]); M[:3, 3] = p; return M

def load(xml="robot_walk.xml"):
    root = ET.parse(os.path.join(MD, xml)).getroot()
    bodies = {}   # name -> dict(parent, T_parent(4x4, zero pose), joint(name, range), servos[], bearings[], parts[], children[])
    order = []
    def walk(b, parent, Tw):
        name = b.get("name"); order.append(name)
        Tp = _T(b.get("pos"), b.get("quat"))
        Twb = Tw @ Tp
        j = b.find("joint")
        d = dict(name=name, parent=parent, T_parent=Tp, T_world=Twb, servos=[], bearings=[], parts=[], children=[],
                 joint=None if j is None else dict(name=j.get("name"), range_deg=np.degrees([float(x) for x in j.get("range").split()])))
        for g in b.findall("geom"):
            if g.get("class") != "visual": continue
            M = _T(g.get("pos"), g.get("quat")); m = g.get("mesh")
            if m == "xl330": d["servos"].append(dict(center=M[:3, 3], axis=M[:3, 0], long=M[:3, 2], T=M))
            elif "bearing" in m: d["bearings"].append(dict(center=M[:3, 3], axis=M[:3, 2], kind="22x16x4" if "22x16" in m else "15x?x3", T=M))
            else: d["parts"].append(dict(mesh=m, T=M))
        for s in b.findall("site"): d.setdefault("sites", {})[s.get("name")] = np.array([float(x) for x in s.get("pos").split()]) * 1000
        bodies[name] = d
        for c in b.findall("body"):
            Tc = _T(c.get("pos"), c.get("quat"))
            d["children"].append(dict(name=c.get("name"), T=Tc, origin=Tc[:3, 3], axis=Tc[:3, 2]))
            walk(c, name, Twb)
    walk(root.find("worldbody").find("body"), None, np.eye(4))
    # 舵机的“法兰朝向”：轴线上离它最近的关节原点（子关节原点，或本体自身原点）
    for name, d in bodies.items():
        for s in d["servos"]:
            cands = [(c["name"], c["origin"]) for c in d["children"]] + ([(name + ":self", np.zeros(3))] if d["joint"] else [])
            best = None
            for jn, o in cands:
                v = o - s["center"]; along = v @ s["axis"]; perp = np.linalg.norm(v - along * s["axis"])
                if perp < 1.0 and (best is None or abs(along) < abs(best[1])): best = (jn, along)
            if best is None:      # 下颚舵机：walk 模型里下颚不是关节，法兰朝向未知，先按 +axis
                s["drives"] = None; s["front"] = s["axis"]; s["dist"] = 0.0
            else:
                s["drives"] = best[0]; s["front"] = s["axis"] * np.sign(best[1]); s["dist"] = abs(best[1])
    return bodies, order

def mesh(name, scale=1000.0):
    m = trimesh.load(os.path.join(ASSETS, name + ".stl")); m.apply_scale(scale); return m

if __name__ == "__main__":
    B, order = load()
    for n in order:
        d = B[n]
        print(f"== {n:18s} parent={d['parent']}  joint={d['joint']['name'] if d['joint'] else '-'}")
        for s in d["servos"]:
            print(f"   servo c={np.round(s['center'],1)} front={np.round(s['front'],0)} long={np.round(s['long'],0)} drives={s['drives']} dist={s['dist']:.1f}")
        for b in d["bearings"]: print(f"   bearing {b['kind']} c={np.round(b['center'],1)} axis={np.round(b['axis'],0)}")
        for c in d["children"]: print(f"   child {c['name']} @ {np.round(c['origin'],1)} axis={np.round(c['axis'],0)}")
