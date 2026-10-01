#!/usr/bin/env python3
"""惰轮（背面副轴）双支撑检查：逐颗舵机看两端的轴颈各被谁抓着。

S288 两端是同一个接口（手册尺寸图 p2）：Ø14×3 凸台 + 6-Ø1.7⌄3.0(MAX) M2 自攻孔 + 中心一颗十字螺钉。
孔位两端共用 S["horn_r"]（2026-09-13 实物定案 5.25，见 s288.py 里 horn_r 的注释）。
法兰端(+x)是输出，背面那个盘(-x)是**自由转动的惰轮**——接上从动件它就跟着转。
把从动件同时接到两端，弯矩由两个轴颈分担；只接一端，全压在法兰这一个塑料轴颈上。

本脚本只筛查端面附近材料，不验证连接拓扑、承载或完整运动。
壳体 C、输出 O、自由惰轮 I：O–I 刚接可形成双支撑；C–I 刚接仅固定惰轮，
不必锁 O；同时存在 O–I、C–I 刚接才形成锁死闭环。材料接近不等于刚接。
从动件端面接近仅报候选；还需六孔/咬入/坐面、从动件连通、壳体转动间隙及装配验证。
载体端面接近报待复核（非零退出），不能由此推断锁死或其他惰轮方案不可能。

    ./.venv/bin/python tools/cad/idlercheck.py
"""
import glob, math, os, sys
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import duckstructure as D
from duckstructure import s288
from duckstructure.s288 import S

TOL = 0.35          # 贴合面离惰轮端面多近算"面接触"
PROBE = 8.0         # 沿 -x 往外探多远
NAZ = 12            # 每个半径上的方位取样数

# 舵机由谁"抱着"（载体侧，与 asmcheck.CARRIER 同一张表）
CARRIER = {("trunk_base", 0): "trunk", ("trunk_base", 1): "trunk",
           ("yaw2roll", 0): "yaw2roll", ("upper_leg_left", 0): "upper_leg", ("upper_leg_left", 1): "upper_leg",
           ("leg", 0): "lower_leg", ("neck", 0): "neck", ("neck", 1): "neck",
           ("yaw_roll_motion", 0): "yrm", ("jaw_soft", 1): "head_bracket",
           ("bearing_roll", 0): "yaw2roll_R", ("upper_leg_right", 0): "upper_leg_R",
           ("upper_leg_right", 1): "upper_leg_R", ("leg_2", 0): "lower_leg_R"}
# MJCF 连杆名 → 我们的打印件名（一个连杆可能由几个件组成）
LINK2PART = {"trunk_base": ["trunk", "shell_L", "shell_R"], "yaw2roll": ["yaw2roll"], "bearing_roll": ["yaw2roll_R"],
             "hip_l": ["hip"], "hip_l_2": ["hip_R"], "upper_leg_left": ["upper_leg"], "upper_leg_right": ["upper_leg_R"],
             "leg": ["lower_leg"], "leg_2": ["lower_leg_R"], "ankle_left": ["ankle_foot", "ankle_rear_arm"],
             "ankle_right": ["ankle_foot_R", "ankle_rear_arm_R"], "neck": ["neck"], "neck_pitch": ["neck_pitch"],
             "yaw_roll_motion": ["yrm", "head_journal"], "jaw_soft": ["head_bracket", "head_clamp", "head_bottom_shell"]}

def load_parts():
    out = {}
    for p in sorted(glob.glob("cad/duck_s288/placed/*.stl")):
        n = os.path.basename(p)[:-4]
        if not n.startswith("servo_"):
            out[n] = trimesh.load(p, process=True)      # process=True 必须：否则布尔报 "Not all meshes are volumes"
    return out

def seat_radii():
    """惰轮贴合面上"一定有料"的取样半径：中心让位孔外侧一圈 + 6 孔（Ø idler_hole_d）内缘以内一圈 + 6 孔外缘与惰轮盘（Ø rear_boss_d）边缘的中点一圈
    + **盘边一圈 r=rear_boss_d/2**（09-14 加：T02 压进髋偏航惰轮盘边缘 0.019 mm³ 时前三圈最外只到 6.775、够不到 r 7.0 的盘边）。
    全部从 S 推出来，horn_r 改了会自动跟着走（5.25 → 2.80 / 3.65 / 6.775 / 7.0；旧 4.75 → 2.80 / 3.15 / 6.10）。
    外圈 09-13 起取"孔外半径"(rear_boss_d/2 + horn_r + idler_hole_d/2)/2：旧的 rear_boss_d/2−0.8=6.2 在 horn_r=5.25 时落进孔带
    3.95..6.55，12 个方位里 6 个打进孔，只能靠 hits ≥ 2·NAZ 的门槛蒙混。断言外圈在孔带外 ≥0.1 且在盘内 ≥0.1，免得再退化。"""
    r_hole_out = S["horn_r"] + S["idler_hole_d"] / 2                  # 6.55：6 孔外缘
    r_out = (S["rear_boss_d"] / 2 + r_hole_out) / 2                   # 6.775：孔外缘与盘边(7.0)的中点
    assert r_out >= r_hole_out + 0.1 and r_out <= S["rear_boss_d"] / 2 - 0.1, (r_out, r_hole_out)
    return (S["idler_center_d"] / 2 + 0.3,
            S["horn_r"] - S["idler_hole_d"] / 2 - 0.3,
            r_out,
            S["rear_boss_d"] / 2)

def segments(mesh, R, radii):
    """在给定半径的环上沿舵机 -x 打射线，返回 (最靠内的料的 x, 覆盖了几个取样点)。
    取样点打空 = 那个方位真的没料，不是"跳过不算"。"""
    o, ex, ey, ez = R[:3, 3], R[:3, 0], R[:3, 1], R[:3, 2]
    x_in, hits = None, 0
    for r in radii:
        for k in range(NAZ):
            a = 2 * math.pi * k / NAZ
            org = o + ey * (r * math.cos(a)) + ez * (r * math.sin(a)) + ex * (-60.0)
            loc = mesh.ray.intersects_location(ray_origins=np.array([org]), ray_directions=np.array([ex]),
                                               multiple_hits=True)[0]
            if not len(loc):
                continue
            t = np.dot(loc - o, ex)
            t = t[t < -10.0]                             # 只看惰轮这一侧
            if not len(t):
                continue
            hits += 1
            m = float(np.max(t))                          # 最靠内（最接近惰轮）的那个面
            x_in = m if x_in is None else max(x_in, m)
    return x_in, hits

if __name__ == "__main__":
    parts = load_parts()
    XI = s288.x_idler_face()
    RR = seat_radii()
    print(f"惰轮端面（舵机局部）x = {XI:.2f}；贴合面取样半径 {[round(r, 2) for r in RR]}（由 horn_r 推出）；面接触容差 ±{TOL}")
    print(f"{'关节':17s}{'从动件离惰轮':22s}{'挡路的非从动件':24s}结论")
    bad = 0
    for body in D.ORDER:
        for i, sv in enumerate(D.B[body]["servos"]):
            if sv["drives"] is None:
                continue
            R = D.sfw(body, i)
            tag = sv["drives"].replace(":self", "")
            link = D.B[body]["parent"] if sv["drives"].endswith(":self") else sv["drives"]
            driven = [p for p in LINK2PART.get(link, []) if p in parts]
            host = CARRIER.get((body, i))
            # 从动件 / 非从动件各自最靠内（最接近惰轮）的那个面，分开算——只报"最近的那个"会把两种情况混掉
            dv_best, ot_best = None, None
            for name, m in parts.items():
                x_in, hits = segments(m, R, RR)
                if x_in is None or hits < NAZ * 2:            # 覆盖不到一圈的算擦边，不算贴合面
                    continue
                slot = "dv" if name in driven else "ot"
                cur = dv_best if slot == "dv" else ot_best
                if cur is None or x_in > cur[1]:
                    if slot == "dv": dv_best = (name, x_in)
                    else: ot_best = (name, x_in)
            dv_s = f"{dv_best[0]} 差{XI - dv_best[1]:+.2f}" if dv_best else "(全空)"
            ot_s = f"{ot_best[0]} 差{XI - ot_best[1]:+.2f}" if ot_best else "(无)"
            # proximity is not fastening topology; never label this as a proven lock.
            if ot_best and ot_best[0] == host and XI - ot_best[1] < TOL:
                r = "⚠ 载体材料接近惰轮；检查擦碰/夹紧及刚接拓扑"; bad += 1
            elif dv_best and abs(XI - dv_best[1]) <= TOL:
                r = "△ 从动件坐面接近；双支撑须另验紧固/连通/承载"
            elif ot_best and (dv_best is None or ot_best[1] > dv_best[1]):
                r = f"○ {ot_best[0]}(非从动件) 位于通路；可研究开窗/叉架"
            elif dv_best:
                r = f"○ 从动件就在这儿，差 {XI - dv_best[1]:.2f} → 可接"
            else:
                r = "○ 惰轮前方全空"
            print(f"{tag:17s}{dv_s:22s}{ot_s:24s}{r}")
    print(f"\n载体端面接近待复核: {bad}；本检查不判定锁死或双支撑合格")
    sys.exit(1 if bad else 0)
