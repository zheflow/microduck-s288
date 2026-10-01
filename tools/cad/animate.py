# Blender 无头：上色 + 动画。用法：blender -b --python tools/cad/animate.py -- <placed_dir> <out_dir> [frames]
import bpy, sys, os, json, math, mathutils
argv = sys.argv[sys.argv.index("--") + 1:]
PD, OUT = argv[0], argv[1]; NF = int(argv[2]) if len(argv) > 2 else 180
os.makedirs(OUT, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
ax = json.load(open(os.path.join(PD, "axes.json")))
S = 0.001

def mat(name, rgb, rough=0.5, metal=0.0):
    m = bpy.data.materials.new(name); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1); b.inputs["Roughness"].default_value = rough; b.inputs["Metallic"].default_value = metal
    return m
M = dict(yellow=mat("yellow", (0.95, 0.72, 0.08), 0.45), orange=mat("orange", (0.95, 0.35, 0.05), 0.5), dark=mat("dark", (0.12, 0.12, 0.13), 0.6),
         white=mat("white", (0.92, 0.92, 0.9), 0.35), eye=mat("eye", (0.15, 0.85, 0.95), 0.2), face=mat("face", (0.22, 0.24, 0.26), 0.5), screen=mat("screen", (0.02, 0.02, 0.03), 0.15), green=mat("green", (0.1, 0.5, 0.2), 0.6), blue=mat("blue", (0.15, 0.25, 0.6), 0.6))
COLOR = dict(chassis="dark", deck="dark", neck="dark", head_top="white", head_bottom="yellow", head_jaw="yellow", head_face="face", head_support="dark", head_bracket="dark", head_soft="yellow", head_mouth="yellow", wheelR="yellow", wheelL="yellow", board="green", battery="blue", body_top="white", body_band="yellow", caster="dark", collar="yellow")

def load(name):
    bpy.ops.wm.stl_import(filepath=os.path.join(PD, name + ".stl"))
    o = bpy.context.selected_objects[0]; o.name = name
    o.scale = (S, S, S); bpy.ops.object.transform_apply(scale=True)
    o.data.materials.append(M[COLOR[name]])
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35)) if hasattr(bpy.ops.object, "shade_smooth_by_angle") else bpy.ops.object.shade_smooth()
    return o
objs = {n: load(n) for n in COLOR}

def empty(name, loc):
    e = bpy.data.objects.new(name, None); e.location = loc; sc.collection.objects.link(e); return e
def parent(child, par):
    bpy.context.view_layer.update()
    child.parent = par; child.matrix_parent_inverse = par.matrix_world.inverted()

car = empty("car", (0, 0, 0))
for n in ("chassis", "deck", "board", "battery", "body_top", "body_band", "caster", "collar"): parent(objs[n], car)
yaw = empty("yaw", tuple(v * S for v in ax["yaw_axis"])); parent(yaw, car); parent(objs["neck"], yaw)
pitch = empty("pitch", tuple(v * S for v in ax["pitch_axis"])); parent(pitch, yaw)
for n in ("head_top", "head_bottom", "head_jaw", "head_face", "head_support", "head_bracket", "head_soft", "head_mouth"): parent(objs[n], pitch)
wR = empty("wR", (ax["wheel_x"] * S, -0.068, ax["wheel_axis_z"] * S)); wL = empty("wL", (ax["wheel_x"] * S, 0.068, ax["wheel_axis_z"] * S))
parent(wR, car); parent(wL, car); parent(objs["wheelR"], wR); parent(objs["wheelL"], wL)
# 表情屏（1.9" 170x320，显示区 42.72x22.7）+ 上方摄像头镜头
lx, ly, lz = [v * S for v in ax["lcd"]]
aw, ah = [v * S for v in ax["lcd_act"]]
def plate(name, w, h, loc, m, par, thick=0.0006):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.active_object; o.name = name; o.scale = (thick, w, h); o["s0"] = (thick, w, h)
    o.data.materials.append(M[m]); parent(o, par); return o
plate("screen", aw, ah, (lx + 0.0016, ly, lz), "screen", pitch)                 # 屏面（黑）
ew = aw * 0.20; eh = ah * 0.42; ex = lx + 0.0021
_eyes = [plate(f"eye{sy}", ew, eh, (ex, ly + sy * aw * 0.20, lz + ah * 0.05), "eye", pitch, 0.0004)
         for sy in (1, -1)]                                                     # 两只简笔画眼睛（青色）
cx_, cy_, cz_ = [v * S for v in ax["cam"]]
bpy.ops.mesh.primitive_cylinder_add(radius=ax["cam_d"] * S / 2, depth=0.004,
                                    location=(cx_ + 0.0016, cy_, cz_), rotation=(0, math.pi / 2, 0))
_c = bpy.context.active_object; _c.name = "camlens"; _c.data.materials.append(M["screen"]); parent(_c, pitch)
pmin, pmax = [math.radians(v) for v in ax["pitch_range"]]

# 地面 + 灯光 + 相机
bpy.ops.mesh.primitive_plane_add(size=3, location=(0, 0, 0)); g = bpy.context.active_object; g.data.materials.append(mat("ground", (0.75, 0.73, 0.68), 0.8))
sun = bpy.data.lights.new("sun", "SUN"); sun.energy = 3.0; so = bpy.data.objects.new("sun", sun); so.rotation_euler = (math.radians(50), math.radians(10), math.radians(40)); sc.collection.objects.link(so)
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; w.node_tree.nodes["Background"].inputs[0].default_value = (0.85, 0.88, 0.92, 1); w.node_tree.nodes["Background"].inputs[1].default_value = 1.0
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam")); sc.collection.objects.link(cam); sc.camera = cam; cam.data.lens = 50
tgt = empty("target", (0, 0, 0.07)); parent(tgt, car)
tc = cam.constraints.new("TRACK_TO"); tc.target = tgt; tc.track_axis = "TRACK_NEGATIVE_Z"; tc.up_axis = "UP_Y"

# 动画：0-2 s 向前开并右转，2-4 s 停下左右看+点头，4-6 s 原地转圈跳舞
FPS = 30; sc.render.fps = FPS; sc.frame_start = 1; sc.frame_end = NF
r = ax["wheel_r"] * S; track = 0.136
x = y = th = 0.0; spinR = spinL = 0.0
for f in range(1, NF + 1):
    t = f / FPS
    if t < 2.0:   vR, vL = 0.12, 0.09
    elif t < 4.0: vR = vL = 0.0
    else:
        k = (t - 4.0); vR, vL = 0.10 * math.sin(k * math.pi), -0.10 * math.sin(k * math.pi)
    v = (vR + vL) / 2; om = (vR - vL) / track
    x += v * math.cos(th) / FPS; y += v * math.sin(th) / FPS; th += om / FPS
    spinR += vR / r / FPS; spinL += vL / r / FPS
    car.location = (x, y, 0); car.rotation_euler = (0, 0, th); car.keyframe_insert("location", frame=f); car.keyframe_insert("rotation_euler", frame=f)
    wR.rotation_euler = (0, -spinR, 0); wL.rotation_euler = (0, -spinL, 0); wR.keyframe_insert("rotation_euler", frame=f); wL.keyframe_insert("rotation_euler", frame=f)
    if t < 2.0:   yv, pv = 0.15 * math.sin(t * 3), 0.0
    elif t < 4.0: yv, pv = 0.9 * math.sin((t - 2) * math.pi), pmax * max(0, math.sin((t - 2) * 2 * math.pi))
    else:         yv, pv = 0.6 * math.sin((t - 4) * 4 * math.pi), (pmax if math.sin((t - 4) * 8 * math.pi) > 0 else -pmin) * abs(math.sin((t - 4) * 8 * math.pi)) * 0.8
    yaw.rotation_euler = (0, 0, yv); pitch.rotation_euler = (0, pv, 0)
    yaw.keyframe_insert("rotation_euler", frame=f); pitch.keyframe_insert("rotation_euler", frame=f)
    blink = 1.0 if (f % 60) not in (28, 29, 30) else 0.15          # 眨眼：把两只眼睛压扁
    for _e in _eyes:
        s0 = _e["s0"]; _e.scale = (s0[0], s0[1], s0[2] * blink); _e.keyframe_insert("scale", frame=f)
    ang = th + math.radians(-60 + 200 * (t / 6)); dist = 0.40                       # 相机从左前绕到右后
    cam.location = (x + dist * math.cos(ang), y + dist * math.sin(ang), 0.22); cam.keyframe_insert("location", frame=f)

sc.render.resolution_x, sc.render.resolution_y = 960, 720
try:
    sc.render.engine = "BLENDER_EEVEE_NEXT"
except Exception:
    try: sc.render.engine = "BLENDER_EEVEE"
    except Exception: sc.render.engine = "BLENDER_WORKBENCH"
try:
    sc.eevee.taa_render_samples = 16
    sc.eevee.use_shadows = True
except Exception: pass
sc.render.image_settings.file_format = "PNG"; sc.render.filepath = os.path.join(OUT, "f_")
if os.environ.get("STILL_V6"):
    for _o in (cam, tgt, car, yaw, pitch, wR, wL, *_eyes):
        _o.animation_data_clear()
    sc.frame_set(1); car.location=(0,0,0); car.rotation_euler=(0,0,0)
    for _e in _eyes: _e.scale = tuple(_e["s0"])

    for tag,(yv,pv,cam_loc,lens) in {
        "v6_face":  (0.0, 0.0, (0.52,0.0,0.150), 60),
        "v6_iso":   (math.radians(22), pmax*0.5, (0.40,0.34,0.27), 42),
        "v6_side":  (0.0, 0.0, (0.047,0.36,0.140), 55),
    }.items():
        yaw.rotation_euler=(0,0,yv); pitch.rotation_euler=(0,pv,0)
        cam.location=cam_loc; cam.data.lens=lens; tgt.location=(0.047,0,0.138) if tag=="v6_side" else (0.06,0,0.125)
        sc.render.filepath=os.path.join(OUT, tag+".png"); bpy.ops.render.render(write_still=True)
elif os.environ.get("STILL_NECK"):
    # 脖套对比：头抬到 +20°、偏航 30°，跟用户截的那帧同角度
    sc.frame_set(1); car.location = (0, 0, 0); car.rotation_euler = (0, 0, 0)
    yaw.rotation_euler = (0, 0, math.radians(30)); pitch.rotation_euler = (0, pmax, 0)
    cam.location = (0.30, 0.26, 0.20); cam.data.lens = 55
    tgt.location = (0.04 * S * 0 + 0.04, 0, 0.095)
    for tag, vis in (("neck_with", False), ("neck_without", True)):
        objs["collar"].hide_render = vis
        sc.render.filepath = os.path.join(OUT, tag + ".png"); bpy.ops.render.render(write_still=True)
    objs["collar"].hide_render = False
elif os.environ.get("STILL_REAR"):
    sc.frame_set(1); car.location = (0, 0, 0); car.rotation_euler = (0, 0, 0); yaw.rotation_euler = (0, 0, 0.5); pitch.rotation_euler = (0, 0, 0)
    cam.location = (-0.42, 0.22, 0.05); cam.data.lens = 40
    sc.render.filepath = os.path.join(OUT, "rear_low.png"); bpy.ops.render.render(write_still=True)
else:
    bpy.ops.render.render(animation=True)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "duckcar.blend"))
print("done", sc.render.engine)
