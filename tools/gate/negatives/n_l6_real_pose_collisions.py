#!/usr/bin/env python3
"""Negative fixtures for strict physical collision acceptance and evidence integrity."""
from __future__ import annotations
import json
import sys
import tempfile
import numpy as np
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))
import _l6_scene  # sets the real Gate import paths
import core
import l6_motion as L6
import policy_pose_compare as compare
import policy_pose_evidence as evidence
from _harness import findings, record, result

NAME = '真实实体碰撞不因原版共有而豁免；缺失/篡改/过期证据不得绿；hr50：极限段进重训约束、轻碰要体积+深度都在原版 p25 内、深度算不出不得绿'


def run():
    assertions, reds = [], []
    with tempfile.TemporaryDirectory(prefix='duck_real_pose_negative_') as temp:
        root = Path(temp)
        mapping = root / 'map.json'
        mapping.write_text(json.dumps({'body_for_part': {'a': 'A', 'b': 'B'}}))
        names = evidence.joint_names_from_mjcf()
        row = dict(mode='stand', case='test', step=0, pose_deg={j: 0. for j in names}, hits=[])
        for tag in ['clear', 'new', 'shared', 'worse', 'stale', 'mode', 'missing',
                    'tampered', 'deleted_field', 'loose_threshold', 'missing_pose', 'empty',
                    'coarse_grid', 'partial', 'both_omit_joint', 'unapproved_source',
                    'retrain', 'light_ok', 'light_deep', 'light_err', 'light_big', 'bad_policy']:
            d = root / tag; d.mkdir()
            original = d / 'orig'; original.mkdir()
            steps = d / 'steps.npz'
            np.savez(steps, **{'joint_names': np.asarray(names),
                              'stand/test/qpos_deg': np.zeros((1, len(names))),
                              'stand/test/prefall_n': np.asarray(1)})
            vol = 1.0 if tag in ('light_ok', 'light_deep', 'light_err') else (2.5 if tag == 'light_big' else 22.73)
            ours = dict(row, hits=[] if tag in ('clear', 'empty') else [['a', 'b', vol]])
            orig = dict(row, hits=[['a', 'b', 3. if tag == 'worse' else 22.73]] if tag in ('shared', 'worse') else [])
            (d / 'poses.jsonl').write_text(json.dumps(ours) + '\n')
            (original / 'poses.jsonl').write_text(json.dumps(orig) + '\n')
            summary = dict(steps_file=str(steps), grid_deg=2.5, modes_in_scope=['stand'],
                           poses_checked=1, poses_with_hits=int(bool(ours['hits'])),
                           complete=True, partial=False, expected_poses=1,
                           poses_sha256=compare.sha(d / 'poses.jsonl'),
                           fingerprint=evidence.generation_fingerprint(L6.PLACED, steps, mapping, ['stand'], 2.5, .05))
            if tag == 'stale': summary['fingerprint']['placed_sha256'] = 'old'
            if tag == 'mode': summary['modes_in_scope'] = []
            if tag == 'coarse_grid': summary['grid_deg'] = 90.
            if tag == 'partial': summary['complete'] = False; summary['partial'] = True
            if tag == 'both_omit_joint':
                ours['pose_deg'] = dict(ours['pose_deg']); orig['pose_deg'] = dict(orig['pose_deg'])
                del ours['pose_deg'][names[0]]; del orig['pose_deg'][names[0]]
                (d / 'poses.jsonl').write_text(json.dumps(ours) + '\n')
                (original / 'poses.jsonl').write_text(json.dumps(orig) + '\n')
                summary['poses_sha256'] = compare.sha(d / 'poses.jsonl')
            sp = d / 'summary.json'; sp.write_text(json.dumps(summary))
            c = compare.analyze(d / 'poses.jsonl', original / 'poses.jsonl', {'a': 'A', 'b': 'B'}, {'a': 'A', 'b': 'B'}, .05)
            if tag == 'tampered': c['ours_pairs'] = {}; c['poses_with_collisions'] = 0
            if tag == 'deleted_field': del c['defect_pairs']
            if tag == 'loose_threshold': c['min_mm3'] = 1.
            if tag == 'missing_pose': (original / 'poses.jsonl').write_text('')
            if tag == 'empty':
                (original / 'poses.jsonl').write_text(''); (d / 'poses.jsonl').write_text('')
            c['fingerprint'] = dict(ours_poses_sha256=compare.sha(d / 'poses.jsonl'),
                orig_poses_sha256=compare.sha(original / 'poses.jsonl'),
                ours_summary_sha256=compare.sha(sp), ours_map_sha256=compare.sha(mapping),
                orig_map_sha256=compare.sha(mapping), tolerances_sha256=compare.sha(compare.TOLERANCES),
                script_sha256=compare.sha(L6._REAL_POSES_COMPARE_SCRIPT), orig_dir=str(original))
            if tag != 'missing': (d / 'compare.json').write_text(json.dumps(c))
            real_pol = L6._load_real_pose_policy()
            pol = dict(real_pol, segs=set(real_pol['segs']) | ({('stand', 'test')} if tag == 'retrain' else set()))
            def fake_depth(scene, solid_pairs, pose, _t=tag):
                if _t == 'light_err': raise RuntimeError('boolean failed')
                return (0.30 if _t == 'light_ok' else 1.20), [dict(pair=sp, depth_mm=0.30 if _t == 'light_ok' else 1.20) for sp in solid_pairs]
            def bad_policy():
                raise ValueError('policy unreadable')
            with patch.object(L6, '_REAL_POSES', sp), patch.object(compare, 'OURS_MAP', mapping), patch.object(compare, 'ORIG_MAP', mapping), \
                    patch.object(L6, '_load_real_pose_policy', bad_policy if tag == 'bad_policy' else (lambda: pol)), \
                    patch.object(L6, '_light_contact_depth', fake_depth), patch.object(L6, '_DepthScene', lambda bfp: None):
                r = core.LayerResult(6, 'negative')
                approved = steps if tag != 'unapproved_source' else mapping
                L6._real_pose_collisions(r, {'modes_in_scope': ['stand'], 'pose_steps_source':
                    {'path': str(approved), 'sha256': compare.sha(approved)}}, .05)
            fs = findings(r, subject='_capability', check='real_pose_collisions')
            rc = findings(r, subject='_capability', check='real_pose_retrain_constraints')
            if tag == 'clear':
                ok = len(fs) == 1 and fs[0].state == 'PASS' and fs[0].evidence_n == 1
                assertions.append(dict(claim='register_evidence_inputs', ok=all(
                    L6._rel(path) in r.inputs for path in (sp, d/'compare.json', d/'poses.jsonl',
                                                          original/'poses.jsonl', steps, Path(evidence.__file__)))))
                score = core.build_scorecard([r], core.hash_inputs(r.inputs), [], 0., layers_run=[6])
                steps.write_bytes(steps.read_bytes() + b'changed source')
                inherited = core.build_scorecard([core.LayerResult(2, 'other layer')], {}, [], 0.,
                                                 prev=score, layers_run=[2])
                assertions.append(dict(claim='other-layer run marks changed NPZ evidence stale',
                    ok=inherited['cells']['L6/_capability']['state'] == 'STALE'))
            elif tag in ('new', 'shared', 'worse', 'light_deep', 'light_err', 'light_big'):
                # 普通段：超出原版 p25（体积 > 2.29，或深度 > 0.77，或深度算不出）→ BLOCK；原版共有不豁免
                ok = len(fs) == 1 and fs[0].state == 'FAIL' and fs[0].severity == 'BLOCK' and fs[0].measured == 1
                reds.extend(record(f) for f in fs)
            elif tag == 'light_ok':
                # 普通段轻碰（1.0 mm³、深 0.30）→ 不阻断，但要逐条写出来
                ok = (len(fs) == 1 and fs[0].state == 'PASS' and fs[0].measured == 0 and '轻碰' in (fs[0].detail or '')
                      and len(rc) == 1 and rc[0].state == 'PASS')
            elif tag == 'retrain':
                # 极限段 22.73 mm³ → 不阻断；进重训约束（INFO），清单里有这一对
                ok = (len(fs) == 1 and fs[0].state == 'PASS' and fs[0].measured == 0
                      and len(rc) == 1 and rc[0].severity == 'INFO' and rc[0].measured['n_pairs'] == 1
                      and rc[0].measured['items'][0]['pair'] == 'A×B')
            else:
                ok = len(fs) == 1 and fs[0].state == 'FAIL' and fs[0].measured is None
            assertions.append(dict(claim=tag, ok=ok, detail=fs[0].detail if fs else 'no finding'))
    out = result(NAME, '共有/加重/新增实体交叠BLOCK；无碰撞对照PASS；坏证据unknown；极限段→重训约束INFO；轻碰需体积与深度都在p25内，深度算不出BLOCK；口径文件坏→unknown',
                 all(a['ok'] for a in assertions),
                 '; '.join(f"{a['claim']}={a['ok']}" for a in assertions), reds)
    out['assertions'] = assertions
    return out


if __name__ == '__main__':
    r = run(); print(json.dumps(r, ensure_ascii=False, indent=1)); sys.exit(0 if r['passed'] else 1)
