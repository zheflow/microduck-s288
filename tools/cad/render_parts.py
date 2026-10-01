# blender -b --python tools/cad/render_parts.py -- <dir with meta.json> <out_prefix>
import bpy, sys, os, json, math, mathutils
argv=sys.argv[sys.argv.index("--")+1:]; D, prefix = argv[0], argv[1]
bpy.ops.wm.read_factory_settings(use_empty=True); sc=bpy.context.scene
meta=json.load(open(os.path.join(D,"meta.json")))
mn=[1e9]*3; mx=[-1e9]*3
for it in meta:
    bpy.ops.wm.stl_import(filepath=os.path.join(D,it["file"])); o=bpy.context.selected_objects[0]
    o.scale=(0.001,)*3; bpy.ops.object.transform_apply(scale=True)
    m=bpy.data.materials.new(it["file"]); m.use_nodes=True; b=m.node_tree.nodes["Principled BSDF"]
    r,g,bb,a=it["rgba"]; b.inputs["Base Color"].default_value=(r,g,bb,1); b.inputs["Roughness"].default_value=0.5
    o.data.materials.append(m)
    for i in range(3):
        mn[i]=min(mn[i],o.bound_box[0][i]); mx[i]=max(mx[i],o.bound_box[6][i])
c=[(mn[i]+mx[i])/2 for i in range(3)]; r=max(mx[i]-mn[i] for i in range(3))
w=bpy.data.worlds.new("w"); sc.world=w; w.use_nodes=True; w.node_tree.nodes["Background"].inputs[0].default_value=(0.9,0.9,0.92,1)
sun=bpy.data.lights.new("s","SUN"); sun.energy=3; so=bpy.data.objects.new("s",sun); so.rotation_euler=(math.radians(50),0,math.radians(30)); sc.collection.objects.link(so)
cam=bpy.data.objects.new("cam",bpy.data.cameras.new("cam")); sc.collection.objects.link(cam); sc.camera=cam
cam.data.type="ORTHO"; cam.data.ortho_scale=r*1.4; cam.data.clip_end=100
sc.render.engine="BLENDER_EEVEE"; sc.render.resolution_x=900; sc.render.resolution_y=700
views={"iso":(60,0,35),"front":(90,0,90),"side":(90,0,0),"top":(0,0,0),"iso2":(60,0,-35)}
for name,(a,b_,g) in views.items():
    rot=(math.radians(a),math.radians(b_),math.radians(g)); cam.rotation_euler=rot
    dv=mathutils.Vector((0,0,1)); dv.rotate(mathutils.Euler(rot)); cam.location=mathutils.Vector(c)+dv*r*3
    sc.render.filepath=f"{prefix}_{name}.png"; bpy.ops.render.render(write_still=True)
print("ok")
