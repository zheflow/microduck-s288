"""Independent read-back checks for restored bearing assemblies, in mm/mm^3.

Checks actual exported holes, seats, stops, assembly paths and screw access.
Nominal geometry results never substitute for PETG fit or load tests.

hr06 (2026-09-17): generalised from the hip-roll-only script (.bak_hr05) to a
station spec so the hip-pitch 6704 module (L11/L12/L13, HP) is audited with the
same probes as the hip-roll 6703 module (L08/L09/L10, HR). Probe geometry and
thresholds are unchanged for the hip-roll station.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import duckstructure as D
from duckstructure.bearing_rebuild import HR
from duckstructure.hip_pitch_bearing_rebuild import HP
from assembly_audit import solid, intersection, path_peak, bearing_specs


def stations():
    """Per-station spec. ears = keeper/cradle screw centres in servo-local (y,z);
    seat_x = plane of the keeper screw heads; pilot_depth = blind pilot in the host."""
    out=[]
    for side,body,host,driven,suffix in [
            ('left','yaw2roll','yaw2roll','hip',''),
            ('right','bearing_roll','yaw2roll_R','hip_R','_R')]:
        out.append(dict(station='hip_roll',side=side,body=body,idx=0,host=host,driven=driven,
            keeper=None,cradle=host,sleeve='hip_roll_sleeve'+suffix,      # hr08：整体座 = L01 本体；无压盖
            bn='bearing_'+side+'_hip_roll',
            od=HR['od'],bore=HR['bore'],width=HR['width'],x0=HR['x0'],seat_d=HR['seat_d'],
            cradle_front=HR['seat_x1'],ray_x1=HR['seat_x1']-1.0,open_top=True,ears=[],   # 座壁最后 0.8 mm 内侧有缺口（HR.medial_notch），射线面退到 x0+width−1
            seat_x=None,receiver_x=None,pilot_d=None,pilot_depth=None,screw_l=None,
            hub_d=HR['journal_d'],sleeve_end=HR['sleeve_end'],inner_face_d=HR['inner_face_d'],
            screw_stack=HR['screw_stack'],horn_screw_l=HR['horn_screw_l'],
            stage_context=['trunk','servo__trunk_base_yaw2roll','servo__trunk_base_bearing_roll',
                           'bearing_left_hip_yaw','bearing_right_hip_yaw'],
            driven_context=[],
            # L02 arrives with the whole pitch station already screwed on (F06 done); they slide on together.
            driven_extra=['hip_pitch_sleeve'+suffix,'bearing_'+side+'_hip_pitch',
                          'upper_leg'+suffix,'servo__'+('upper_leg_left_upper_leg_left' if side=='left' else 'upper_leg_right_upper_leg_right')],
            module_before_driven=True))
    for side,body,host,driven,suffix,knee_servo in [
            ('left','upper_leg_left','upper_leg','hip','','servo__upper_leg_left_leg'),
            ('right','upper_leg_right','upper_leg_R','hip_R','_R','servo__upper_leg_right_leg_2')]:
        out.append(dict(station='hip_pitch',side=side,body=body,idx=1,host=host,driven=driven,
            keeper=None,cradle=host,sleeve='hip_pitch_sleeve'+suffix,     # hr07：整体座 = L03 本体；无压盖
            bn='bearing_'+side+'_hip_pitch',
            od=HP['od'],bore=HP['bore'],width=HP['width'],x0=HP['x0'],seat_d=HP['seat_d'],
            cradle_front=HP['seat_x1'],open_top=False,ears=[],
            seat_x=None,receiver_x=None,pilot_d=None,pilot_depth=None,screw_l=None,
            hub_d=HP['journal_d'],sleeve_end=HP['sleeve_end'],inner_face_d=HP['inner_face_d'],
            screw_stack=HP['horn_seat']-D.s288.x_flange_face(),horn_screw_l=HP['horn_screw_l'],
            stage_context=[knee_servo],driven_context=[],module_before_driven=True))
    return out


def audit_station(sp,meshes,shapes,record):
    side=sp['side']; tag=sp['station']+' '+side
    host,driven,keeper,cradle,sleeve,bn=[sp[k] for k in ('host','driven','keeper','cradle','sleeve','bn')]
    integral=keeper is None                       # hr07 髋俯仰：外圈座是载体本体，无压盖
    needed=[n for n in (host,driven,keeper,cradle,sleeve,bn) if n]
    if not all(n in meshes for n in needed):
        record(tag+' declared parts present',False,[n for n in needed if n not in meshes],len(needed)); return
    R=D.sfw(sp['body'],sp['idx']).copy()
    ex,ez=R[:3,0],R[:3,2]
    if integral:
        sy=1 if side=='left' else -1              # 轴对称站，符号只影响探针，不影响结论
    else:
        # Right printed parts are reflections; find which local y sign the exported keeper ears actually occupy.
        cand=[]
        for sy in (1,-1):
            ey=R[:3,1]*sy
            # Off-centre by 1.8: the ear centre is the through hole; 1.8 is inside the ear body (r 2.7..3.4) outside the hole (r 1.2..2.2).
            pts=[R[:3,3]+ex*(sp['seat_x']-.5)+ey*(y+1.8)+ez*z for y,z in sp['ears']]
            cand.append(int(meshes[keeper].contains(np.asarray(pts)).sum()))
        sy=1 if cand[0]>=cand[1] else -1
        record(tag+' keeper ear side resolved',max(cand)==len(sp['ears']) and min(cand)==0,dict(hits_by_sign=cand,sy=sy),2)
    ey=R[:3,1]*sy
    def point(x,y=0,z=0): return R[:3,3]+ex*x+ey*y+ez*z
    sn='servo__'+sp['body']+'_'+D.B[sp['body']]['servos'][sp['idx']]['drives'].replace(':self','')
    for n in dict.fromkeys(x for x in (host,driven,keeper,cradle,sleeve) if x):
        m=meshes[n]
        record(tag+' '+n+' connected',len(m.split(only_watertight=False))==1 and m.is_watertight,
               len(m.split(only_watertight=False)),1)
    overlap={n:intersection(shapes[bn],s) for n,s in shapes.items() if n!=bn}
    record(tag+' bearing full envelope',max(overlap.values())<=.05,overlap,len(overlap))
    rays=[]
    # The keeper clamps metal, so the seat wall deliberately ends before
    # the metal front face. Probe inside the retained wall, not its edge.
    for x in np.linspace(sp['x0']+.2,sp.get('ray_x1',sp['cradle_front']-.2),3):
        for angle in np.arange(1.5,360.,3.):
            if sp['open_top'] and 90.-HR['open_top_deg']/2-1e-9<=angle<=90.+HR['open_top_deg']/2+1e-9: continue
            a=np.deg2rad(angle); direction=ey*np.cos(a)+ez*np.sin(a)
            hit=meshes[cradle].ray.intersects_location([point(x)],[direction],multiple_hits=True)[0]
            ts=np.unique(np.round((hit-point(x))@direction,6)) if len(hit) else np.array([])
            ts=ts[ts>=0]
            rays.append(dict(x=x,angle=float(angle),bore=float(2*ts[0]) if len(ts) else None,
                             wall=float(ts[1]-ts[0]) if len(ts)>1 else None))
    record(tag+' retained seat bore and radial wall',
           all(r['bore'] is not None and abs(r['bore']-sp['seat_d'])<.06
               and r['wall'] is not None and r['wall']>=1.1 for r in rays),rays,len(rays))
    if integral:
        # 外圈：−x 由座唇止；+x 无压盖（与膝 6704 同级：靠舵机轴向堆叠 + 座配合，bench BN08 实测旷量）—— 记为已知项，不冒充夹紧。
        rear=intersection(shapes[bn].translate(ex*-.3),shapes[host]); front=intersection(shapes[bn].translate(ex*.3),shapes[host])
        record(tag+' outer race rear stop (lip)',rear>.05,dict(rear_stop_mm3_at_0p3=rear,front_stop_mm3_at_0p3=front,
               front_retention='none by design (knee-class); axial location by servo stack + seat fit — physical BN08'),2)
        clamp={'-1':intersection(shapes[bn].translate(ex*-.02),shapes[host])}
        record(tag+' outer race rear face contact',clamp['-1']>.05,dict(contact_at_002mm_mm3=clamp),1)
        ear_gaps=[]
    else:
      # Metal ring must be stopped in BOTH axial directions by fixed parts.
      stops={str(s):intersection(shapes[bn].translate(ex*.3*s),shapes[cradle]+shapes[keeper]) for s in (-1,1)}
      record(tag+' bearing axial capture',all(v>.05 for v in stops.values()),stops,2)
      clamp={str(sign):intersection(shapes[bn].translate(ex*.02*sign),shapes[n])
           for sign,n in [(-1,cradle),(1,keeper)]}
      ear_gaps=[]
    if not integral:
      for y,z in sp['ears']:
        org=point(0,y+1.8,z)
        spans=[]
        for n in (cradle,keeper):
            hits=meshes[n].ray.intersects_location([org],[ex],multiple_hits=True)[0]
            spans.append(np.sort((hits-org)@ex) if len(hits) else [])
        ear_gaps.append(float(spans[1][0]-spans[0][-1]) if all(len(v)>=2 for v in spans) else None)
      record(tag+' outer race clamp contact and ear bypass',
           all(v>.05 for v in clamp.values()) and all(g is not None and g>=.15 for g in ear_gaps),
           dict(contact_at_002mm_mm3=clamp,ear_gap_mm=ear_gaps,
                scope='Nominal outer-race face clamp; measured race boundaries and torque still required'),2+len(ear_gaps))
    # Probe the two faces of the intended inner race, plus the plastic
    # bypass gap. The bearing manufacturer's actual race-face dimensions
    # remain a separately declared physical/drawing verification.
    def lc(d,t,x,y=0,z=0):
        return solid(D.placed(D.cyl(d,t,(x,sy*y,z),axis='x',sections=128),R))
    inner=lc(sp['inner_face_d'],sp['width'],sp['x0']+sp['width']/2)-lc(sp['bore'],sp['width']+.2,sp['x0']+sp['width']/2)
    contact={name:intersection(inner.translate(ex*.06*sign),shapes[n])
             for name,sign,n in [('rear',-1,sleeve),('front',1,driven)]}
    bypass=intersection(lc(sp['hub_d']-.1,.2,sp['sleeve_end']+.15),shapes[sleeve]+shapes[driven])
    record(tag+' inner race clamp path and plastic bypass',
           all(v>.05 for v in contact.values()) and bypass<=.01,
           dict(contact_at_006mm=contact,plastic_gap_fill_mm3=bypass,
                assumed_inner_race_face_d=sp['inner_face_d'],nominal_gap_mm=.3),3)
    assembly={}
    def path(label,movers,present,direction):
        assembly[label]=dict(movers=movers,present=present,
            result=path_peak({n:shapes[n] for n in movers},{n:shapes[n] for n in present},
                             direction,step=.5,distance=60.))
    path('sleeve_into_bearing_from_rear',[sleeve],[bn],-ex)
    ctx=sp['stage_context']; dctx=sp['driven_context']
    missing=[n for n in ctx+dctx if n not in shapes]
    if missing:
        record(tag+' stage inventory',False,missing,len(ctx+dctx)); return
    if integral:
        # 舵机已侧滑装入 L03；轴承+轴套从法兰侧沿 −ex 推进整体座（拆 = +ex）。
        present=[host,sn]+ctx+[bn,sleeve]
        path('bearing_sleeve_into_seat_over_horn',[bn,sleeve],[host,sn]+ctx,ex)
    else:
        path('bearing_sleeve_into_cradle',[bn,sleeve],[cradle],ex)
        path('keeper_before_driven',[keeper],[cradle,bn,sleeve],ex)
        present=[host,sn]+ctx+[cradle,keeper,bn,sleeve]
        path('complete_module_onto_motor',[cradle,keeper,bn,sleeve],[host,sn]+ctx,ex)
    extra=[n for n in sp.get('driven_extra',[]) if n in shapes]
    if dctx:
        # Driven part already carries its other module: the completed station is offered to it (station moves -ex).
        path('station_off_driven',present,[driven]+dctx,-ex)
    else:
        path('driven_last',[driven]+extra,present,ex)
    record(tag+' ordered axial assembly',all(r['result']['volume_mm3']<=.05 for r in assembly.values()),
           assembly,sum(121*len(r['movers'])*len(r['present']) for r in assembly.values()))
    hole_rows=[]
    for y,z in ([] if integral else sp['ears']):
        center=point(sp['seat_x'],y,z)
        head=solid(D.placed(D.cyl(D.P['m2_head_d'],D.P['m2_head_h'],
                    (sp['seat_x']+D.P['m2_head_h']/2,sy*y,z),axis='x'),R))
        tool=solid(D.placed(D.cyl(4.,40.,(sp['seat_x']+20.,sy*y,z),axis='x'),R))
        head_hits={n:intersection(head,shapes[n]) for n in present}
        tool_hits={n:intersection(tool,shapes[n]) for n in present}
        final_tool_diagnostic={n:intersection(tool,s) for n,s in shapes.items()}
        footprint=[center-ex*.03+(ey*np.cos(a)+ez*np.sin(a))*r
                   for r in (1.3,1.65,1.95) for a in np.linspace(0,2*np.pi,24,endpoint=False)]
        support=meshes[keeper].contains(np.asarray(footprint))
        pilot=solid(D.placed(D.cyl(sp['pilot_d']*.97,sp['pilot_depth']-.06,
                 (sp['receiver_x']-sp['pilot_depth']/2,sy*y,z),axis='x'),R))
        pilot_hit=intersection(pilot,shapes[host])
        teeth=[point(sp['receiver_x']-x,y,z)+(ey*np.cos(a)+ez*np.sin(a))*.95
               for x in (.5,1.5,2.5,3.5) for a in np.linspace(0,2*np.pi,24,endpoint=False)]
        support_thread=meshes[host].contains(np.asarray(teeth))
        hole_rows.append(dict(ear=[y,z],head=head_hits,tool=tool_hits,final_tool_diagnostic=final_tool_diagnostic,
                              thread_support=int(support_thread.sum()),thread_support_n=len(teeth),
                              footprint_supported=int(support.sum()),footprint_n=len(support),
                              pilot_void_mm3=pilot_hit))
    if not integral:
      record(tag+' keeper screws and tools',
           all(max(r['head'].values())<=.05 and max(r['tool'].values())<=.05
               and r['footprint_supported']==r['footprint_n'] and r['pilot_void_mm3']<=.01
               and r['thread_support']==r['thread_support_n']
               for r in hole_rows),hole_rows,len(hole_rows)*(2*len(present)+169))
      stack=sp['seat_x']-sp['receiver_x']
      engagement=sp['screw_l']-stack
      record(tag+' keeper screw nominal depth',0<engagement<=sp['pilot_depth']-.3,
           dict(length=sp['screw_l'],stack=stack,engagement=engagement,pilot=sp['pilot_depth'],
                limitation='Geometric depth only; PETG self-tap pullout and actual race faces unverified'),2)
    flange_rows=[]
    xf=D.S['T']/2+D.S['flange_h']; xseat=xf+sp['screw_stack']
    obstacles=present+[driven]+dctx
    for angle in np.linspace(0,2*np.pi,D.S['horn_n'],endpoint=False):
        y,z=D.S['horn_r']*np.cos(angle),D.S['horn_r']*np.sin(angle)
        head=lc(D.P['m2_head_d'],D.P['m2_head_h'],xseat+D.P['m2_head_h']/2,y,z)
        tool=lc(4.,40.,xseat+20.,y,z)
        hh={n:intersection(head,shapes[n]) for n in obstacles}
        th={n:intersection(tool,shapes[n]) for n in obstacles}
        fp=[point(xseat-.03,y,z)+(ey*np.cos(a)+ez*np.sin(a))*r
            for r in (1.3,1.65,1.95) for a in np.linspace(0,2*np.pi,24,endpoint=False)]
        supported=meshes[driven].contains(np.asarray(fp))
        shaft=lc(2.0,sp['screw_stack']-.04,(xf+xseat)/2,y,z)
        shaft_hit=intersection(shaft,shapes[sleeve]+shapes[driven])
        flange_rows.append(dict(head=hh,tool=th,footprint=int(supported.sum()),n=len(fp),shaft_block_mm3=shaft_hit))
    record(tag+' horn screw heads tools and holes',
           all(max(r['head'].values())<=.05 and max(r['tool'].values())<=.05
               and r['footprint']==r['n'] and r['shaft_block_mm3']<=.01 for r in flange_rows),
           flange_rows,len(flange_rows)*(2*len(obstacles)+73))
    depth=sp['horn_screw_l']-sp['screw_stack']
    record(tag+' horn screw nominal engagement',0<depth<=D.S['horn_depth']-.3,
           dict(length=sp['horn_screw_l'],stack=sp['screw_stack'],engagement=depth,
                blind_max=D.S['horn_depth'],source='s288.py manufacturer depth; actual holding torque unverified'),2)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--station',choices=['hip_roll','hip_pitch'],default=None)
    args=ap.parse_args()
    folder=ROOT/'cad/duck_s288/placed'
    meshes={p.stem:trimesh.load(p,process=True) for p in folder.glob('*.stl')}
    shapes={k:solid(v) for k,v in meshes.items()}
    checks=[]
    def record(name,passed,measured,evidence_n):
        checks.append(dict(name=name,passed=bool(passed and evidence_n>0),
                           measured=measured,evidence_n=int(evidence_n)))
    for sp in stations():
        if args.station and sp['station']!=args.station: continue
        audit_station(sp,meshes,shapes,record)
    sources=list(folder.glob('*.stl'))+list((ROOT/'duckstructure').glob('*.py'))+[Path(__file__)]
    result=dict(passed=all(c['passed'] for c in checks),checks=checks,
                input_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                scope='Hip-roll + hip-pitch (both integral seats, hr08) exported nominal geometry only; head stations audited separately; no physical release.')
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    for c in checks: print(('PASS' if c['passed'] else 'FAIL'),c['name'],'evidence_n='+str(c['evidence_n']))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
