import os
"""件的上下文渲染（无 GPU）：正交投影 + 画家算法 + 双面朗伯着色，直接读 cad/duck_s288/placed/*.stl。

    ./.venv/bin/python tools/cad/render_context.py <输出目录> duck|hip|exploded

  duck      整鸭，指定件高亮（默认 L01 = yaw2roll）
  hip       左髋子装配（躯干隐藏）
  exploded  左髋沿 +y 拉开的爆炸图

2026-09-14 为 L01 试打件说明写的。相机：cam(az, el) 先转台再抬头，世界 z → 屏幕上（写这个的时候
踩过一次坑：直接用 Rx(el) 会把世界 z 映射到屏幕深度，出来是俯视图）。改件列表见 HIP/MODE 分支。
与 render.py / render_parts.py 的区别：那两个出的是整鸭成品渲染图，这个专门做"某个件在哪、和谁装在一起"。"""
import sys, os, glob, numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"]=["Heiti SC","Arial Unicode MS"]; matplotlib.rcParams["axes.unicode_minus"]=False
from matplotlib.collections import PolyCollection
SP, MODE = sys.argv[1], sys.argv[2]
P="cad/duck_s288/placed/"
HIP=[("servo__trunk_base_yaw2roll.stl","#9aa3ad","髋偏航舵机",10.),
     ("bearing_left_hip_yaw.stl","#3f6f9f","6702ZZ",22.),
     ("yaw2roll.stl","#3e9e5c","L01（本件）",34.),
     ("servo__yaw2roll_hip_l.stl","#6f7782","髋横滚舵机",50.),
     ("bearing_left_hip_roll.stl","#3f6f9f","6704ZZ",62.),
     ("hip.stl","#d4a422","L02 髋件",74.)]
def cam(az_deg, el_deg):
    az=np.radians(az_deg); t=-np.radians(90.0-el_deg)     # 先转台，再抬头：世界 z → 屏幕上
    Rz=np.array([[np.cos(az),-np.sin(az),0],[np.sin(az),np.cos(az),0],[0,0,1]])
    Rx=np.array([[1,0,0],[0,np.cos(t),-np.sin(t)],[0,np.sin(t),np.cos(t)]])
    return Rx@Rz
L=np.array([-0.45,-0.4,0.80]); L/=np.linalg.norm(L)
def add(items, R, tris, cols, deps, labs, focus_names=()):
    for fn,col,lab,dy in items:
        f=P+fn
        if not os.path.exists(f): continue
        m=trimesh.load(f, process=True)
        if dy: m.apply_translation((0.,dy,0.))
        V=(R@m.vertices.T).T; T=V[m.faces]; n=(R@m.face_normals.T).T
        fr=n[:,2]>0.0; T=T[fr]; n=n[fr]
        if not len(T): continue
        sh=np.abs(n@L)*0.58+0.42
        base=np.array(matplotlib.colors.to_rgb(col))
        tris.append(T[:,:,:2]); cols.append(np.clip(base[None,:]*sh[:,None],0,1)); deps.append(T[:,:,2].mean(axis=1))
        if lab: labs.append((lab,col,V[:,:2]))
        if fn in focus_names: focus.append(V[:,:2])
tris=[];cols=[];deps=[];labs=[];focus=[]
if MODE=="duck":
    R=cam(-58,14); title="整鸭 · L01 在哪（绿色，左右各一）"
    others=[(os.path.basename(p),"#d8dadd","",0.) for p in sorted(glob.glob(P+"*.stl"))
            if os.path.basename(p) not in ("yaw2roll.stl","yaw2roll_R.stl")]
    add(others,R,tris,cols,deps,[])
    add([("yaw2roll.stl","#2e8b50","L01",0.),("yaw2roll_R.stl","#2e8b50","",0.)],R,tris,cols,deps,labs,
        focus_names=("yaw2roll.stl",))
elif MODE=="mount":                                   # 横滚舵机怎么装进 L01：舵机沿世界 -z 拉下来
    R=cam(128,14); title="髋横滚舵机怎么装进 L01（舵机沿 −z 抽出）"
    J=[("yaw2roll.stl","#3e9e5c","L01（背板 3 mm 朝我们）",(0,0,0)),
       ("servo__yaw2roll_hip_l.stl","#6f7782","髋横滚舵机（本体滑进 L01 的框腔）",(0,0,-34))]
    for fn,col,lab,dv in J:
        if not os.path.exists(P+fn): continue   # 09-17：髋横滚 6704 已取消
        m=trimesh.load(P+fn,process=True); m.apply_translation(dv)
        V=(R@m.vertices.T).T; T=V[m.faces]; n=(R@m.face_normals.T).T
        fr=n[:,2]>0.0; T=T[fr]; n=n[fr]
        sh=np.abs(n@L)*0.58+0.42; base=np.array(matplotlib.colors.to_rgb(col))
        tris.append(T[:,:,:2]); cols.append(np.clip(base[None,:]*sh[:,None],0,1)); deps.append(T[:,:,2].mean(axis=1))
        labs.append((lab,col,V[:,:2])); focus.append(V[:,:2])
elif MODE=="joint":                                   # 横滚关节特写：沿横滚轴(x)拉开
    R=cam(-38,16); title="髋横滚关节 · L01 ↔ L02 怎么连（沿横滚轴拉开）"
    J=[("yaw2roll.stl","#3e9e5c","L01（座 Ø27.15）",0.),
       ("servo__yaw2roll_hip_l.stl","#6f7782","髋横滚舵机（机体在 L01 里）",26.),
       ("bearing_left_hip_roll.stl","#3f6f9f","6704ZZ",44.),
       ("hip.stl","#d4a422","L02（毂 Ø19.85 + 驱动盘 Ø26）",62.)]
    for fn,col,lab,dx in J:
        if not os.path.exists(P+fn): continue   # 09-17：髋横滚 6704 已取消
        m=trimesh.load(P+fn,process=True); m.apply_translation((dx,0.,0.))
        V=(R@m.vertices.T).T; T=V[m.faces]; n=(R@m.face_normals.T).T
        fr=n[:,2]>0.0; T=T[fr]; n=n[fr]
        sh=np.abs(n@L)*0.58+0.42; base=np.array(matplotlib.colors.to_rgb(col))
        tris.append(T[:,:,:2]); cols.append(np.clip(base[None,:]*sh[:,None],0,1)); deps.append(T[:,:,2].mean(axis=1))
        labs.append((lab,col,V[:,:2])); focus.append(V[:,:2])
elif MODE=="hip":
    R=cam(-52,18); title="左髋 · 装配后（躯干已隐藏）"
    add([(a,b,c,0.) for a,b,c,_ in HIP],R,tris,cols,deps,labs,focus_names=tuple(a for a,_,_,_ in HIP))
else:
    R=cam(-90,18); title="左髋 · 爆炸图（沿 +y 向外拉开）"
    add([("trunk.stl","#e0e3e6","",0.)],R,tris,cols,deps,[])
    add(HIP,R,tris,cols,deps,labs,focus_names=tuple(a for a,_,_,_ in HIP))
T=np.vstack(tris); C=np.vstack(cols); D=np.concatenate(deps); o=np.argsort(D)
F=np.vstack(focus) if focus else np.vstack([v for _,_,v in labs])
fig,ax=plt.subplots(figsize=(11,8) if MODE!="duck" else (8,11),dpi=150)
ax.add_collection(PolyCollection(T[o],facecolors=C[o],edgecolors=C[o],linewidths=0.25))
pad=16 if MODE!="duck" else 40
x0,x1,y0,y1=F[:,0].min()-pad,F[:,0].max()+pad,F[:,1].min()-pad,F[:,1].max()+pad
if MODE=="duck":
    A=T.reshape(-1,2); x0,x1,y0,y1=A[:,0].min()-8,A[:,0].max()+8,A[:,1].min()-8,A[:,1].max()+8
ax.set_xlim(x0,x1); ax.set_ylim(y0,y1+(y1-y0)*(0.34 if MODE in ("exploded","joint","mount") else 0.10))
ax.set_aspect("equal"); ax.axis("off"); fig.patch.set_facecolor("#fbfaf7")
span=y1-y0
if MODE in ("exploded","joint","mount"):
    for i,(_,lab,col,v) in enumerate(sorted([(v[:,0].mean(),l,c,v) for l,c,v in labs],key=lambda t:t[0])):
        ax.annotate(lab,xy=(v[:,0].mean(),v[:,1].max()),xytext=(v[:,0].mean(),y1+span*(0.24 if i%2==0 else 0.11)),
            fontsize=12.5,color="#1a1a1a",ha="center",va="bottom",
            arrowprops=dict(arrowstyle="-",color=col,lw=1.6,shrinkA=2,shrinkB=3))
else:
    n=len(labs); 
    for i,(lab,col,v) in enumerate(sorted(labs,key=lambda t:-t[2][:,1].mean())):
        ty=y1-span*0.04-i*span*0.085; tx=x1-span*0.02
        ax.annotate(lab,xy=(v[:,0].mean(),v[:,1].mean()),xytext=(tx,ty),fontsize=12.5,color="#1a1a1a",
            ha="right",va="center",
            arrowprops=dict(arrowstyle="-",color=col,lw=1.5,shrinkA=0,shrinkB=4,connectionstyle="arc3,rad=0.08"))
sx,sy=x0+(x1-x0)*0.03,y0+(y1-y0)*0.03
ax.plot([sx,sx+20],[sy,sy],color="#555",lw=2); ax.text(sx+10,sy+(y1-y0)*0.012,"20 mm",ha="center",fontsize=10,color="#555")
ax.set_title(title,fontsize=16,color="#111",pad=8)
out=f"{SP}/L01_{MODE}.png"; fig.savefig(out,bbox_inches="tight",facecolor="#fbfaf7"); print("→",out)
