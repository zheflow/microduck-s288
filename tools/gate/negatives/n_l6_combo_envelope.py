#!/usr/bin/env python3
"""hr50 v4（2026-09-28）：L6 pair_combination 按原版真实动作包络分类的反例（函数级）。
目标区间外 → workspace（WARN，F-L6-3 不变）；区间内没到过 → 重训约束（按连通块给耦合区间，不并成一个大盒子）；
到过的超轻碰 / 深度超 / 深度算不出 → 违规；到过的轻碰 → 放行；左右镜像按 frozen.yaml:mirror_pairs 并进包络；
极限段样本不许把角落算成"到过"；包络证据坏（sha / mode / 镜像声明 / 口径缺段 / 邻域比网格半步小）→ 抛（上层判 unknown）。
件格子 motion_collision / collision_buckets 用同一分类由 n_l6_target_range_severity（A 工作空间 WARN / B 违规 BLOCK）在整层上验。
hr50 v5（09-28 17:0x）：真口径文件里 ARC01–06 的件对 / 盒 / 上限 / 左右镜像；ARC05/06 盒内有原版真实样本要标"按格点限"，撞点格在真包络里确是"到过"。"""
from __future__ import annotations
import hashlib
import json
import sys
import tempfile
import numpy as np
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))
import _l6_scene  # noqa: F401  sets the real Gate import paths
import l6_motion as L6
from _harness import result

NAME = ('L6 组合碰撞分类：区间外 WARN；区间内没到过 → 重训约束（连通块各自区间）；到过的超轻碰 / 深度超 / 深度算不出 → 违规；'
        '镜像并进包络；极限段不算到过；证据坏 → 抛')
J = ['l1', 'l2', 'r1', 'r2']
PAIRS = [['l1', 'r1', -1], ['l2', 'r2', -1]]


def _env_file(root):
    p = root / 'steps.npz'
    walk = np.array([[0., 0., 0., 0.], [10., 10., 0., 0.], [20., 5., 0., 0.],
                     [0., 0., -30., -30.]])            # 右侧到过 (-30, -30) → 镜像后左侧 (30, 30) 算到过
    ext = np.array([[60., 60., 0., 0.]])               # 极限段走到 (60, 60)
    np.savez(p, **{'joint_names': np.asarray(J),
                   'walk/a/qpos_deg': walk, 'walk/a/prefall_n': np.asarray(4),
                   'stand/ext/qpos_deg': ext, 'stand/ext/prefall_n': np.asarray(1)})
    return p


def _raises(fn):
    try:
        fn()
        return False
    except ValueError:
        return True


def _raises_any(fn):
    try:
        fn()
        return False
    except Exception:                                   # noqa: BLE001  KeyError / RuntimeError / ValueError 都算"算不出"
        return True


def run():
    A = []
    with tempfile.TemporaryDirectory(prefix='duck_combo_env_') as t:
        root = Path(t)
        orig_root = L6.ROOT
        try:
            L6.ROOT = root
            p = _env_file(root)
            sha = hashlib.sha256(p.read_bytes()).hexdigest()
            cap = dict(pose_steps_source=dict(path='steps.npz', sha256=sha), modes_in_scope=['walk', 'stand'],
                       mirror_symmetric=True, mirror_pairs=PAIRS)
            pol = dict(segs={('stand', 'ext')}, v_max=2.2868, d_max=0.7696, combo=dict(reach=5.5, excl=True))
            env = L6._combo_envelope(cap, pol)
            env_nomir = L6._combo_envelope(dict(cap, mirror_symmetric=False), pol)
            A.append(dict(claim='retrain segment excluded from envelope',
                          ok=env[2]['n_raw'] == 4 and not L6._combo_reached(env, 'l1', 60, 'l2', 60, 5.5)))
            A.append(dict(claim='mirror adds right-side samples to left (30,30) only when mirror_symmetric',
                          ok=env[2]['n'] == 8 and L6._combo_reached(env, 'l1', 30, 'l2', 30, 5.5)
                          and not L6._combo_reached(env_nomir, 'l1', 30, 'l2', 30, 5.5)))
            A.append(dict(claim='reached within ±reach on both axes', ok=L6._combo_reached(env, 'l1', 15, 'l2', 10, 5.5)))
            A.append(dict(claim='not reached when one axis is outside ±reach', ok=not L6._combo_reached(env, 'l1', 15, 'l2', 16, 5.5)))
            A.append(dict(claim='unknown joint raises (no guessing)', ok=_raises(lambda: L6._combo_reached(env, 'l1', 0, 'jaw', 0, 5.5))))

            deep = {('a', 'b'): 0.3, ('c', 'd'): 0.9}

            class _FakeScene:
                """与 _DepthScene.depth 同接口：返回 pose_contact_depth.contact_depth 那种 dict（不是数）——
                第一次整层试跑就是在这里栽的（层里把 dict 当数用，276 块深度全部"算不出"）。"""
                def depth(self, a, b, pose):
                    if (a, b) == ('e', 'f'):
                        raise RuntimeError('boom')
                    if (a, b) == ('i', 'j'):
                        return dict(vol_mm3=1.0)                      # 缺 depth_mm
                    return dict(vol_mm3=1.0, depth_mm=deep.get((a, b), 0.1), crop_limited=False)
            scene = _FakeScene()

            def depth_of(pa, pb, pose):
                return L6._combo_depth_mm(scene, pa, pb, pose)
            A.append(dict(claim='depth helper reads depth_mm from contact_depth dict', ok=depth_of('a', 'b', {}) == 0.3))
            A.append(dict(claim='depth helper raises when depth_mm missing', ok=_raises_any(lambda: depth_of('i', 'j', {}))))
            A.append(dict(claim='depth helper raises on scene error', ok=_raises_any(lambda: depth_of('e', 'f', {}))))

            def in_target(pose):                                   # 目标区间：两轴都在 [-70, 70]
                return all(-70 - 1e-6 <= v <= 70 + 1e-6 for v in pose.values())
            rows = [(60., 60., ('x', 'y', 30.0)), (55., 65., ('x', 'y', 12.0)),    # 块 1：没到过 → 重训
                    (-60., 60., ('x', 'y', 8.0)),                                   # 块 2：另一个角落，不许和块 1 并成一个盒子
                    (10., 10., ('a', 'b', 1.0)),                                    # 到过、轻碰
                    (10., 10., ('c', 'd', 1.0)),                                    # 到过、深度超
                    (10., 10., ('g', 'h', 2.5)),                                    # 到过、体积超
                    (10., 10., ('e', 'f', 1.0)),                                    # 到过、深度算不出
                    (80., 10., ('m', 'n', 50.0))]                                   # 目标区间外 → 工作空间
            cls, ret, light, viol, ws, appr0 = L6._classify_combo_rows(rows, 'l1', 'l2', env, pol, depth_of, in_target,
                                                                       (-90., 90.), (-90., 90.), 5.0)
            A.append(dict(claim='no approved list → empty approved rows', ok=appr0 == []))
            A.append(dict(claim='per-row classes aligned with rows',
                          ok=cls == ['retrain', 'retrain', 'retrain', 'light', 'viol', 'viol', 'viol', 'workspace']))
            b1 = [r for r in ret if r['q1'][1] > 0]
            b2 = [r for r in ret if r['q1'][1] < 0]
            A.append(dict(claim='two separate corners → two blocks (no big box over reached poses)',
                          ok=len(ret) == 2 and len(b1) == 1 and len(b2) == 1))
            r = b1[0] if b1 else {}
            A.append(dict(claim='block interval = grid hull padded one step, max volume + depth at max pose',
                          ok=r.get('poses') == 2 and r.get('q1') == ['l1', 55., 60.] and r.get('q2') == ['l2', 60., 65.]
                          and r.get('q1_pad') == ['l1', 50., 65.] and r.get('q2_pad') == ['l2', 55., 70.]
                          and r.get('max_mm3') == 30.0 and r.get('depth_mm') == 0.1 and r.get('real_samples_in_pad_box') == 0))
            A.append(dict(claim='in-envelope light contact passes', ok=[x['pair'] for x in light] == ['a×b']))
            A.append(dict(claim='in-envelope deep / big / depth-error → violation',
                          ok=sorted(x['pair'] for x in viol) == ['c×d', 'e×f', 'g×h']))
            A.append(dict(claim='depth error never green', ok=any(x['pair'] == 'e×f' and '算不出' in x['why'] for x in viol)))
            A.append(dict(claim='outside target range → workspace (F-L6-3), not retrain', ok=[x['pair'] for x in ws] == ['m×n']))
            A.append(dict(claim='no approved list → no approved rows', ok=len(ret) == 2 and cls.count('approved') == 0))
            # hr50 v4：用户批准的重训联动限位 —— 件对 + 关节对 + 盒 + 体积上限四者都对上才放行
            def _e(id_, joints, pair, q1, q2, cap):
                return dict(id=id_, joints=joints, pair=frozenset(pair), q1=q1, q2=q2, cap=cap, date='2026-09-28', by='u', why='w', evidence='')
            approved = [_e('ARC-T1', ['l1', 'l2'], ('g', 'h'), [5., 15.], [5., 15.], 3.0),        # 盒内、2.5 ≤ 3.0 → approved
                        _e('ARC-T2', ['l2', 'l1'], ('c', 'd'), [5., 15.], [5., 15.], 0.5),        # joints 反序也认；1.0 > 0.5 → 超上限 → viol
                        _e('ARC-T3', ['l1', 'l2'], ('e', 'f'), [50., 60.], [50., 60.], 9.0),      # 盒外 → viol；本条 unused
                        _e('ARC-T4', ['l1', 'q9'], ('g', 'h'), [5., 15.], [5., 15.], 9.0)]        # 关节对不匹配 → 不算
            cls2, ret2b, light2, viol2, ws2, appr2 = L6._classify_combo_rows(rows, 'l1', 'l2', env, pol, depth_of, in_target,
                                                                              (-90., 90.), (-90., 90.), 5.0, approved)
            A.append(dict(claim='approved: in box & ≤ cap → approved (not viol)',
                          ok=cls2 == ['retrain', 'retrain', 'retrain', 'light', 'viol', 'approved', 'viol', 'workspace']
                          and [r['id'] for r in appr2] == ['ARC-T1'] and appr2[0]['pair'] == 'g×h'))
            A.append(dict(claim='approved: in box but over cap → still viol, why names the cap',
                          ok=any(x['pair'] == 'c×d' and '超过批准上限 0.5' in x['why'] for x in viol2)))
            A.append(dict(claim='approved: out of box / wrong joints → viol untouched',
                          ok=any(x['pair'] == 'e×f' for x in viol2) and len(viol2) == 2))
            A.append(dict(claim='approved: other classes unchanged', ok=len(ret2b) == 2 and len(light2) == 1 and len(ws2) == 1))
            # 上限"原样登记 chain 的 6 位小数"：原始浮点只在第 7 位以后超出 → 不算超；第 4 位超出 → 超
            e6 = [_e('ARC-R', ['l1', 'l2'], ('g', 'h'), [5., 15.], [5., 15.], 54.225)]
            A.append(dict(claim='cap compare rounds volume to record precision 4 dp (54.225006 ≤ 54.225 approved; 54.2251 / 54.226 over)',
                          ok=L6._MEASURED_DECIMALS == 4
                          and L6._approved_match(e6, 'l1', 10., 'l2', 10., ('g', 'h', 54.225006)) == (e6[0], False)
                          and L6._approved_match(e6, 'l1', 10., 'l2', 10., ('g', 'h', 54.2251)) == (e6[0], True)
                          and L6._approved_match(e6, 'l1', 10., 'l2', 10., ('g', 'h', 54.2260)) == (e6[0], True)))
            A.append(dict(claim='approved match helper: joints order-insensitive, pair set-based',
                          ok=L6._approved_match(approved, 'l2', 10., 'l1', 10., ('d', 'c', 0.4))[0] is not None
                          and L6._approved_match(approved, 'l1', 10., 'l2', 10., ('g', 'h', 2.5)) == (approved[0], False)
                          and L6._approved_match(approved, 'l1', 10., 'l2', 10., ('c', 'd', 1.0)) == (approved[1], True)
                          and L6._approved_match(approved, 'l1', 10., 'l2', 10., ('e', 'f', 1.0)) == (None, False)))
            # 盒子罩住真实样本要标出来：块 (10..15, 0..5) 外扩后罩住 (10,10) / (20,5)
            _, ret2, _, _, _, _ = L6._classify_combo_rows([(40., -40., ('x', 'y', 5.0))], 'l1', 'l2', env, pol, depth_of,
                                                       in_target, (-90., 90.), (-90., 90.), 45.0)
            A.append(dict(claim='samples inside padded box are counted', ok=len(ret2) == 1 and ret2[0]['real_samples_in_pad_box'] > 0))
            for tag, c2, p2 in (('bad_sha', dict(cap, pose_steps_source=dict(path='steps.npz', sha256='0' * 64)), pol),
                                ('missing_mode', dict(cap, modes_in_scope=['walk', 'fly']), pol),
                                ('mirror_without_pairs', dict(cap, mirror_pairs=[]), pol),
                                ('mirror_pair_unknown_joint', dict(cap, mirror_pairs=[['l1', 'jaw', -1]]), pol),
                                ('no_combo_policy', cap, dict(pol, combo=None))):
                A.append(dict(claim=f'{tag} raises', ok=_raises(lambda c2=c2, p2=p2: L6._combo_envelope(c2, p2))))
        finally:
            L6.ROOT = orig_root
    # 口径文件：真文件有 pair_combination 段且 reach = 5.5；reach 小于网格半步要被拒
    pol_real = L6._load_real_pose_policy()
    # 按关节对合并：外扩盒相交的块并成一区，不相交的分开；件对与最大交集并进区里
    blocks = [dict(joints='a+b', pair='p×q', q1_pad=['a', 0., 10.], q2_pad=['b', 0., 10.], max_mm3=5.0, poses=2),
              dict(joints='a+b', pair='r×s', q1_pad=['a', 10., 20.], q2_pad=['b', 5., 15.], max_mm3=7.0, poses=3),      # 与上一块相接
              dict(joints='a+b', pair='p×q', q1_pad=['a', 50., 60.], q2_pad=['b', 50., 60.], max_mm3=1.0, poses=1),     # 隔开
              dict(joints='c+d', pair='p×q', q1_pad=['c', 0., 10.], q2_pad=['d', 0., 10.], max_mm3=2.0, poses=1)]       # 另一关节对
    mg = L6._merge_retrain_blocks(blocks)
    big = next((m for m in mg if m['joints'] == 'a+b' and m['n_blocks'] == 2), None)
    A.append(dict(claim='merge: touching blocks of one joint pair → one region with union box and both pairs',
                  ok=len(mg) == 3 and big is not None and big['q1'] == ['a', 0., 20.] and big['q2'] == ['b', 0., 15.]
                  and [p['pair'] for p in big['pairs']] == ['r×s', 'p×q'] and big['n_cells'] == 5 and big['max_mm3'] == 7.0))
    A.append(dict(claim='merge: disjoint block and other joint pair stay separate',
                  ok=sum(1 for m in mg if m['joints'] == 'a+b') == 2 and sum(1 for m in mg if m['joints'] == 'c+d') == 1))
    A.append(dict(claim='real policy file has pair_combination reach 5.5', ok=bool(pol_real.get('combo')) and pol_real['combo']['reach'] == 5.5))
    A.append(dict(claim='real policy file has approved list ARC01–06 with joints/pair/box/cap',
                  ok=[e['id'] for e in pol_real.get('approved') or []] == ['ARC01', 'ARC02', 'ARC03', 'ARC04', 'ARC05', 'ARC06']
                  and all(len(e['joints']) == 2 and len(e['pair']) == 2 and e['q1'][0] <= e['q1'][1] and e['q2'][0] <= e['q2'][1] and e['cap'] > 0
                          for e in pol_real['approved'])))
    # hr50 v5（2026-09-28 17:0x 用户定）：ARC05/06 大腿撞壳 @ 髋偏航 ∓10° / 横滚 ±22° —— 件对、盒、上限、左右镜像都要对上；
    # 盒内有原版真实样本（alpha_stand/body_all_min (−8.7, 17.6)）→ 条目 why 必须标"按格点限"；真包络里撞点格确是"到过"（所以走批准而不是 retrain），
    # 偏航 ∓15 那一列没到过（靠 retrain 块，不靠本条）。
    import yaml as _yaml
    ap = {e['id']: e for e in pol_real.get('approved') or []}
    a5, a6 = ap.get('ARC05'), ap.get('ARC06')
    A.append(dict(claim='ARC05/06: shell×upper_leg, yaw box ∓[5,15], roll box ±[17,22], caps 2.5867 / 7.4978',
                  ok=a5 is not None and a6 is not None
                  and a5['joints'] == ['left_hip_yaw', 'left_hip_roll'] and a5['pair'] == frozenset(('shell_L', 'upper_leg'))
                  and a5['q1'] == [-15.0, -5.0] and a5['q2'] == [17.0, 22.0] and a5['cap'] == 2.5867
                  and a6['joints'] == ['right_hip_yaw', 'right_hip_roll'] and a6['pair'] == frozenset(('shell_R', 'upper_leg_R'))
                  and a6['q1'] == [5.0, 15.0] and a6['q2'] == [-22.0, -17.0] and a6['cap'] == 7.4978))
    fz_real = _yaml.safe_load((L6.ROOT / 'tools/gate/data/frozen.yaml').read_text(encoding='utf-8'))
    cap_real = fz_real['capability_envelope']
    mir = {p[0]: (p[1], float(p[2])) for p in cap_real['mirror_pairs']}

    def _mirror_ok(left, right):
        if [mir.get(j, (None,))[0] for j in left['joints']] != right['joints']:
            return False
        for q, j in ((left['q1'], left['joints'][0]), (left['q2'], left['joints'][1])):
            want = sorted([mir[j][1] * q[0], mir[j][1] * q[1]])
            got = right['q1'] if right['joints'][0] == mir[j][0] else right['q2']
            if [round(x, 6) for x in got] != [round(x, 6) for x in want]:
                return False
        return True
    A.append(dict(claim='mirrored entries ARC01/02, ARC03/04, ARC05/06 are sign-mirrors per frozen.yaml:mirror_pairs',
                  ok=all(ap.get(l) is not None and ap.get(r) is not None and _mirror_ok(ap[l], ap[r])
                         for l, r in (('ARC01', 'ARC02'), ('ARC03', 'ARC04'), ('ARC05', 'ARC06')))))
    appr_real = pol_real.get('approved') or []
    A.append(dict(claim='real cells: (-10,22) 2.5867 → ARC05 ok; 2.5868 → over cap; (10,-22) 7.4978 → ARC06 ok; (-20,22) → no box',
                  ok=a5 is not None and a6 is not None
                  and L6._approved_match(appr_real, 'left_hip_yaw', -10., 'left_hip_roll', 22., ('shell_L', 'upper_leg', 2.5867)) == (a5, False)
                  and L6._approved_match(appr_real, 'left_hip_yaw', -10., 'left_hip_roll', 22., ('upper_leg', 'shell_L', 2.5868)) == (a5, True)
                  and L6._approved_match(appr_real, 'right_hip_roll', -22., 'right_hip_yaw', 10., ('shell_R', 'upper_leg_R', 7.4978)) == (a6, False)
                  and L6._approved_match(appr_real, 'left_hip_yaw', -20., 'left_hip_roll', 22., ('shell_L', 'upper_leg', 1.0)) == (None, False)))
    A.append(dict(claim='ARC05/06 why marks grid-point limiting (real sample body_all_min inside the box)',
                  ok=a5 is not None and a6 is not None and all('按格点限' in e['why'] for e in (a5, a6))))
    env_real = L6._combo_envelope(cap_real, pol_real)
    names_r, S_r = list(env_real[0]), env_real[1]
    iy, ir = names_r.index('left_hip_yaw'), names_r.index('left_hip_roll')
    n_in_box = int(((S_r[:, iy] >= -15.) & (S_r[:, iy] <= -5.) & (S_r[:, ir] >= 17.) & (S_r[:, ir] <= 22.)).sum())
    reach = pol_real['combo']['reach']
    A.append(dict(claim='real envelope: samples inside ARC05 box exist (grid-point limiting needed); (-10,22)/(10,-22) reached; (-15,22) not reached (retrain column)',
                  ok=n_in_box > 0
                  and L6._combo_reached(env_real, 'left_hip_yaw', -10., 'left_hip_roll', 22., reach)
                  and L6._combo_reached(env_real, 'right_hip_yaw', 10., 'right_hip_roll', -22., reach)
                  and not L6._combo_reached(env_real, 'left_hip_yaw', -15., 'left_hip_roll', 22., reach)))
    import yaml
    from unittest.mock import patch
    raw = yaml.safe_load(L6._REAL_POSE_POLICY.read_text(encoding='utf-8'))
    raw['pair_combination']['reach_deg'] = 1.0
    raw_ok = yaml.safe_load(L6._REAL_POSE_POLICY.read_text(encoding='utf-8'))
    bad_entries = {
        'approved entry missing cap rejected': [{k: v for k, v in raw_ok['approved_retrain_constraints'][0].items() if k != 'approved_up_to_mm3'}],
        'approved entry reversed interval rejected': [dict(raw_ok['approved_retrain_constraints'][0], q1=[-20.0, -25.0])],
        'approved entry missing why rejected': [{k: v for k, v in raw_ok['approved_retrain_constraints'][0].items() if k != 'why'}],
        'approved duplicate id rejected': [raw_ok['approved_retrain_constraints'][0], raw_ok['approved_retrain_constraints'][0]],
    }
    with tempfile.TemporaryDirectory(prefix='duck_combo_pol_') as t2:
        bad = Path(t2) / 'p.yaml'
        bad.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding='utf-8')
        with patch.object(L6, '_REAL_POSE_POLICY', bad):
            A.append(dict(claim='reach below half grid step rejected', ok=_raises(L6._load_real_pose_policy)))
        for claim, entries in bad_entries.items():
            b2 = dict(raw_ok); b2['approved_retrain_constraints'] = entries
            bad.write_text(yaml.safe_dump(b2, allow_unicode=True), encoding='utf-8')
            with patch.object(L6, '_REAL_POSE_POLICY', bad):
                A.append(dict(claim=claim, ok=_raises(L6._load_real_pose_policy)))
    out = result(NAME, '区间外 WARN；区间内没到过 → 重训约束（连通块）；到过轻碰放行、深/大/算不出违规；镜像并入；极限段不算到过；证据坏抛',
                 all(a['ok'] for a in A), '; '.join(f"{a['claim']}={a['ok']}" for a in A), [])
    out['assertions'] = A
    return out


if __name__ == '__main__':
    r = run(); print(json.dumps(r, ensure_ascii=False, indent=1)); sys.exit(0 if r['passed'] else 1)
