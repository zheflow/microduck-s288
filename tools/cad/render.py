# Blender 无头渲染：blender -b --python tools/cad/render.py -- <stl> <out_prefix>
import bpy, sys, math, os
argv = sys.argv[sys.argv.index("--") + 1:]
stl, prefix = argv[0], argv[1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.stl_import(filepath=stl)
obj = bpy.context.selected_objects[0]
obj.scale = (0.001, 0.001, 0.001); bpy.ops.object.transform_apply(scale=True)
dims = obj.dimensions; c = [(obj.bound_box[0][i] + obj.bound_box[6][i]) / 2 for i in range(3)]
r = max(dims) * 1.1
scene = bpy.context.scene
scene.render.engine = "BLENDER_WORKBENCH"
scene.display.shading.light = "STUDIO"; scene.display.shading.color_type = "SINGLE"
scene.display.shading.single_color = (0.85, 0.75, 0.3); scene.display.shading.show_cavity = True
scene.render.resolution_x = 1000; scene.render.resolution_y = 750
cam_data = bpy.data.cameras.new("cam"); cam_data.type = "ORTHO"; cam_data.ortho_scale = r * 1.6
cam = bpy.data.objects.new("cam", cam_data); scene.collection.objects.link(cam); scene.camera = cam
views = {"iso": (math.radians(60), 0, math.radians(35)), "front": (math.radians(90), 0, math.radians(90)),
         "side": (math.radians(90), 0, 0), "top": (0, 0, 0)}
for name, rot in views.items():
    cam.rotation_euler = rot
    import mathutils
    d = mathutils.Vector((0, 0, 1)); d.rotate(mathutils.Euler(rot))
    cam.location = mathutils.Vector(c) + d * r * 3
    scene.render.filepath = f"{prefix}_{name}.png"; bpy.ops.render.render(write_still=True)
print("rendered", prefix)
