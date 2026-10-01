#!/usr/bin/env python3
"""Compare identical poses. Upstream collisions diagnose differences, never exempt ours."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
D = ROOT / 'tools/gate/out/policy_steps_2026-09-16'
OURS_MAP = ROOT / 'tools/sim/cad_geometry_manifest.json'
ORIG_MAP = ROOT / 'tools/gate/out/orig_placed_2026-09-16/body_for_part.json'
TOLERANCES = ROOT / 'tools/gate/data/tolerances.yaml'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numerical_tolerance():
    v = yaml.safe_load(TOLERANCES.read_text())['feature_check_tolerances']['static_intersection_mm3']['max']
    if not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
        raise ValueError('Invalid Gate intersection tolerance')
    return float(v)


def load(path, bfp, min_mm3):
    out = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        key = (r['mode'], r['case'], r['step'])
        if key in out:
            raise ValueError(f'duplicate pose: {key}')
        pose = r['pose_deg']
        if not isinstance(pose, dict) or not pose or any(
                not isinstance(v, (float, int)) or not math.isfinite(v) for v in pose.values()):
            raise ValueError(f'invalid pose: {key}')
        pairs = {}
        for a, b, v in r['hits']:
            if not isinstance(v, (float, int)) or not math.isfinite(v) or v < 0:
                raise ValueError(f'invalid collision volume: {key}: {v}')
            if a not in bfp or b not in bfp:
                raise ValueError(f'unmapped solid: {a}, {b}')
            ba, bb = bfp[a], bfp[b]
            if v <= min_mm3 or ba == bb:
                continue
            k = '×'.join(sorted((ba, bb)))
            d = pairs.setdefault(k, {'v': 0., 'parts': set(), 'solid_pairs': set()})
            d['v'] = max(d['v'], v)
            d['parts'].update({a, b})
            d['solid_pairs'].add('×'.join(sorted((a, b))))
        out[key] = (pairs, pose)
    return out


def analyze(ours_path, orig_path, ours_bfp, orig_bfp, min_mm3):
    if not math.isfinite(min_mm3) or min_mm3 < 0:
        raise ValueError('invalid collision threshold')
    ours, orig = load(ours_path, ours_bfp, min_mm3), load(orig_path, orig_bfp, min_mm3)
    if not ours or set(ours) != set(orig):
        raise ValueError(f'pose coverage differs or empty: ours={len(ours)}, orig={len(orig)}; '
                         f'only ours={len(set(ours)-set(orig))}, only orig={len(set(orig)-set(ours))}')
    defect, shared, orig_only, all_ours, worsened = {}, {}, {}, {}, {}
    n_def, n_hit = 0, 0

    def add(dest, k, d, key, pose):
        s = dest.setdefault(k, {'poses': 0, 'max_mm3': 0., 'modes': {},
                               'our_parts': set(), 'solid_pairs': set(), 'example': None})
        s['poses'] += 1
        s['modes'][key[0]] = s['modes'].get(key[0], 0) + 1
        s['our_parts'] |= d['parts']; s['solid_pairs'] |= d['solid_pairs']
        if d['v'] > s['max_mm3']:
            s['max_mm3'] = d['v']
            s['example'] = dict(mode=key[0], case=key[1], step=key[2], pose_deg=pose)
        return s

    for key in sorted(ours):
        po, pose = ours[key]; pg, original_pose = orig[key]
        if set(pose) != set(original_pose) or any(
                not math.isclose(pose[j], original_pose[j], abs_tol=1e-8, rel_tol=0) for j in pose):
            raise ValueError(f'different joint tuples at {key}')
        n_hit += bool(po); n_def += bool(set(po) - set(pg))
        for k, d in po.items():
            add(all_ours, k, d, key, pose)
            if k not in pg:
                add(defect, k, d, key, pose)
                continue
            s = shared.setdefault(k, {'poses': 0, 'max_ours': 0., 'max_orig': 0., 'modes': {}})
            s['poses'] += 1
            s['max_ours'] = max(s['max_ours'], d['v']); s['max_orig'] = max(s['max_orig'], pg[k]['v'])
            s['modes'][key[0]] = s['modes'].get(key[0], 0) + 1
            delta = d['v'] - pg[k]['v']
            if delta > min_mm3:
                w = add(worsened, k, d, key, pose)
                if delta > w.get('max_increase_mm3', 0):
                    w.update(max_increase_mm3=delta, ours_at_max_increase_mm3=d['v'],
                             orig_at_max_increase_mm3=pg[k]['v'],
                             increase_example=dict(mode=key[0], case=key[1], step=key[2]))
        for k, d in pg.items():
            if k not in po:
                s = orig_only.setdefault(k, {'poses': 0, 'max_orig': 0.})
                s['poses'] += 1; s['max_orig'] = max(s['max_orig'], d['v'])
    for dest in (all_ours, defect, worsened):
        for s in dest.values():
            s['our_parts'] = sorted(s['our_parts']); s['solid_pairs'] = sorted(s['solid_pairs'])
    return dict(schema_version=2, poses_compared=len(ours), min_mm3=min_mm3,
                poses_with_defect=n_def, poses_with_collisions=n_hit,
                acceptance='absolute_collision; upstream_shared_is_not_exempt',
                defect_pairs=defect, ours_pairs=all_ours, shared_pairs=shared,
                worsened_pairs=worsened, orig_only_pairs=orig_only)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--min-mm3', type=float, default=None, help='Diagnostic override; Gate requires its configured tolerance')
    ap.add_argument('--ours', type=Path, default=D)
    ap.add_argument('--orig', type=Path, default=D / 'orig')
    ap.add_argument('--out', type=Path, help='Separate output to preserve earlier audit artifacts')
    a = ap.parse_args()
    tol = numerical_tolerance() if a.min_mm3 is None else a.min_mm3
    r = analyze(a.ours / 'poses.jsonl', a.orig / 'poses.jsonl',
                json.loads(OURS_MAP.read_text())['body_for_part'],
                json.loads(ORIG_MAP.read_text())['body_for_part'], tol)
    r['fingerprint'] = dict(ours_poses_sha256=sha(a.ours / 'poses.jsonl'),
                           orig_poses_sha256=sha(a.orig / 'poses.jsonl'),
                           ours_summary_sha256=sha(a.ours / 'summary.json'),
                           ours_map_sha256=sha(OURS_MAP), orig_map_sha256=sha(ORIG_MAP),
                           tolerances_sha256=sha(TOLERANCES), script_sha256=sha(Path(__file__)),
                           ours_dir=str(a.ours.resolve()), orig_dir=str(a.orig.resolve()))
    out = a.out or (a.ours / 'compare.json')
    out.write_text(json.dumps(r, ensure_ascii=False, indent=1), encoding='utf-8')
    print(json.dumps({k: v for k, v in r.items() if k not in (
        'fingerprint', 'ours_pairs', 'defect_pairs', 'shared_pairs', 'worsened_pairs', 'orig_only_pairs')}, ensure_ascii=False))
    print(f"physical body pairs={len(r['ours_pairs'])}; new={len(r['defect_pairs'])}; worsened={len(r['worsened_pairs'])}; output={out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
