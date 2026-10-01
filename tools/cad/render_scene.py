# Blender 无头渲染多件装配：blender -b --python tools/cad/render_scene.py -- <placed_dir> <out_prefix> [zoom_center_x,y,z,radius] [view,view..]
import bpy, sys, math, os, glob, mathutils
argv = sys.argv[sys.argv.index("--") + 1:]
pdir, prefix = argv[0], argv[1]
zoom = [float(v) for v in argv[2].split(",")] if len(argv) > 2 and argv[2] != "-" else None
views_sel = argv[3].split(",") if len(argv) > 3 else ["iso", "front", "side", "top"]
bpy.ops.wm.read_factory_settings(use_empty=True)
palette = [(0.95, 0.75, 0.2), (0.3, 0.7, 0.95), (0.85, 0.35, 0.3), (0.4, 0.85, 0.45), (0.8, 0.5, 0.9), (0.95, 0.55, 0.2), (0.5, 0.9, 0.9), (0.9, 0.9, 0.4)]
files = sorted(glob.glob(os.path.join(pdir, "*.stl")))
incl = argv[4].split(",") if len(argv) > 4 else None
if incl: files = [f for f in files if any(k in os.path.basename(f) for k in incl)]
lo = [1e9] * 3; hi = [-1e9] * 3; k = 0
for f in files:
    bpy.ops.wm.stl_import(filepath=f)
    o = bpy.context.selected_objects[0]
    o.scale = (0.001,) * 3; bpy.ops.object.transform_apply(scale=True)
    name = os.path.basename(f)
    col = (0.25, 0.25, 0.28) if "__" in name else palette[k % len(palette)]
    if "__" not in name: k += 1
    if "sole" in name: col = (0.15, 0.15, 0.15)
    if "orig_" in name: col = (0.9, 0.9, 0.9)
    if "head_bottom_shell" in name: col = (0.9, 0.9, 0.9)
    if name.startswith("bearing_"): col = (0.6, 0.62, 0.65)
    m = bpy.data.materials.new(name); m.diffuse_color = (*col, 1.0); o.data.materials.append(m)
    for v in o.bound_box:
        w = o.matrix_world @ mathutils.Vector(v)
        for i in range(3): lo[i] = min(lo[i], w[i]); hi[i] = max(hi[i], w[i])
c = [(lo[i] + hi[i]) / 2 for i in range(3)]; r = max(hi[i] - lo[i] for i in range(3)) * 0.6
if zoom: c = [v / 1000 for v in zoom[:3]]; r = zoom[3] / 1000
sc = bpy.context.scene; sc.render.engine = "BLENDER_WORKBENCH"
sc.display.shading.light = "STUDIO"; sc.display.shading.color_type = "MATERIAL"; sc.display.shading.show_cavity = True
sc.display.shading.show_object_outline = True
sc.render.resolution_x = 1100; sc.render.resolution_y = 850
cd = bpy.data.cameras.new("cam"); cd.type = "ORTHO"; cd.ortho_scale = r * 2.2
# 单件只有几十毫米；默认100mm近裁面会把相机前的整块小件裁没。
cd.clip_start = max(r * 0.01, 1e-6)
cam = bpy.data.objects.new("cam", cd); sc.collection.objects.link(cam); sc.camera = cam
views = {"iso": (math.radians(62), 0, math.radians(38)), "iso2": (math.radians(62), 0, math.radians(-135)), "front": (math.radians(90), 0, math.radians(90)),
         "side": (math.radians(90), 0, 0), "top": (0, 0, 0), "back": (math.radians(90), 0, math.radians(-90)), "bottom": (math.radians(180), 0, 0)}
for name in views_sel:
    rt = views[name]; cam.rotation_euler = rt
    d = mathutils.Vector((0, 0, 1)); d.rotate(mathutils.Euler(rt))
    cam.location = mathutils.Vector(c) + d * r * 4; cd.clip_end = r * 20
    sc.render.filepath = f"{prefix}_{name}.png"; bpy.ops.render.render(write_still=True)
print("rendered", prefix)
