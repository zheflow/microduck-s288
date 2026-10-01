# 整鸭上色渲染（参考 Microduck 官方 Cream + Orange 配色）：blender -b --python tools/cad/render_duck_color.py -- <placed_dir> <out_prefix> [views]
import bpy, sys, math, os, glob, mathutils
argv = sys.argv[sys.argv.index("--") + 1:]
pdir, prefix = argv[0], argv[1]
views_sel = argv[2].split(",") if len(argv) > 2 else ["iso", "front", "side"]
bpy.ops.wm.read_factory_settings(use_empty=True)
CREAM = (0.93, 0.89, 0.80); ORANGE = (0.95, 0.45, 0.10); GRAPHITE = (0.16, 0.16, 0.18); STEEL = (0.55, 0.55, 0.58); DARK = (0.10, 0.10, 0.10); NAVY = (0.13, 0.16, 0.34)
def color_for(name):
    n = name.lower()
    if n.startswith("servo__"): return GRAPHITE
    if "battery" in n: return NAVY                        # 3S 电池占位
    if "sole" in n: return DARK                           # TPU 鞋底
    if "face_part" in n or "noenoeil" in n or "lens" in n: return DARK   # 脸板（屏幕）
    if "jaw" in n or "mouth" in n: return ORANGE          # 嘴
    if "ankle_foot" in n or n.startswith("foot"): return ORANGE          # 脚
    if "card_" in n: return STEEL                         # 垫片卡
    return CREAM                                          # 壳、腿、躯干、头
files = sorted(glob.glob(os.path.join(pdir, "*.stl")))
lo = [1e9] * 3; hi = [-1e9] * 3
for f in files:
    bpy.ops.wm.stl_import(filepath=f)
    o = bpy.context.selected_objects[0]
    o.scale = (0.001,) * 3; bpy.ops.object.transform_apply(scale=True)
    name = os.path.basename(f); col = color_for(name)
    m = bpy.data.materials.new(name); m.diffuse_color = (*col, 1.0); m.roughness = 0.6; o.data.materials.append(m)
    pass
    for v in o.bound_box:
        w = o.matrix_world @ mathutils.Vector(v)
        for i in range(3): lo[i] = min(lo[i], w[i]); hi[i] = max(hi[i], w[i])
c = [(lo[i] + hi[i]) / 2 for i in range(3)]; r = max(hi[i] - lo[i] for i in range(3)) * 0.55
# 地板
bpy.ops.mesh.primitive_plane_add(size=2.0, location=(c[0], c[1], lo[2]))
fl = bpy.context.active_object; fm = bpy.data.materials.new("floor"); fm.diffuse_color = (0.85, 0.85, 0.85, 1); fl.data.materials.append(fm)
sc = bpy.context.scene; sc.render.engine = "BLENDER_WORKBENCH"
sc.display.shading.light = "STUDIO"; sc.display.shading.color_type = "MATERIAL"; sc.display.shading.show_cavity = True
sc.display.shading.show_shadows = True; sc.display.shading.show_object_outline = False
sc.render.resolution_x = 1400; sc.render.resolution_y = 1100; sc.render.film_transparent = False
sc.world = bpy.data.worlds.new("w"); sc.world.color = (1, 1, 1)
cd = bpy.data.cameras.new("cam"); cd.type = "PERSP"; cd.lens = 60
cam = bpy.data.objects.new("cam", cd); sc.collection.objects.link(cam); sc.camera = cam
views = {"iso": (math.radians(72), 0, math.radians(35)), "iso2": (math.radians(72), 0, math.radians(-145)), "front": (math.radians(85), 0, math.radians(90)),
         "side": (math.radians(85), 0, 0), "back": (math.radians(85), 0, math.radians(-90))}
for name in views_sel:
    rt = views[name]; cam.rotation_euler = rt
    d = mathutils.Vector((0, 0, 1)); d.rotate(mathutils.Euler(rt))
    cam.location = mathutils.Vector(c) + d * r * 5.2; cd.clip_end = r * 40
    sc.render.filepath = f"{prefix}_{name}.png"; bpy.ops.render.render(write_still=True)
print("rendered", prefix)
