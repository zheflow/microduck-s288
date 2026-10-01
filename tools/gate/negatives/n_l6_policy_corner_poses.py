#!/usr/bin/env python3
"""hr50 v6（2026-09-28 用户定 C）：策略包络角点姿态记录（_policy/policy_extreme_poses）按 pair_combination 同一口径分类的反例（函数级）。
"到过" = 存在原版样本在角点姿态的**全部**轴上都 ≤ reach（切比雪夫，姿态里没写的关节按 0）；目标区间外 → workspace；没到过 → retrain
（带深度、姿态去零）；到过 → 轻碰 / 用户批准盒（按条目的两条关节在姿态里的角度判盒）/ 违规（超轻碰、深度超、深度算不出、盒内超上限）；
目标区间分不出 → unknown。件格子 / 汇总里 pose_class 与 combo_class 同权（_row_block_flag）。口径文件：真文件有 policy_extreme_poses.classify =
pair_combination；没有该段 → 加载器给 None（层判 unknown）；classify 不是 pair_combination → 拒。旧代码没有这些函数 → 本反例红。"""
from __future__ import annotations
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

NAME = ('L6 角点姿态记录按 hr50 v6 分类：全轴到过判定；区间外 WARN、没到过 → 重训约束、到过 → 轻碰 / 批准盒 / 违规；'
        'pose_class 在件格子与汇总里与 combo_class 同权；口径段缺 → None、错 → 拒')


def _raises(fn):
    try:
        fn()
    except Exception:  # noqa: BLE001
        return True
    return False


def run() -> dict:
    A = []
    names = ['a', 'b', 'c']
    S = np.array([[0., 0., 0.], [10., 10., 10.], [20., 5., 0.]])
    env = (names, S, {'n': 3})
    R = 5.5
    A.append(dict(claim='reached: all axes within reach of one sample → True; one axis 6° off → False',
                  ok=L6._pose_reached(env, {'a': 12., 'b': 8., 'c': 14.}, R) and not L6._pose_reached(env, {'a': 12., 'b': 8., 'c': 16.}, R)))
    A.append(dict(claim='reached: joints missing from the pose count as 0 (sample (20,5,0))',
                  ok=L6._pose_reached(env, {'a': 20., 'b': 5.}, R) and not L6._pose_reached(env, {'a': 20., 'b': 5., 'c': 5.6}, R)))
    A.append(dict(claim='reached: two axes near two different samples but no single sample near all axes → False',
                  ok=not L6._pose_reached(env, {'a': 20., 'b': 10., 'c': 10.}, R)))
    pol = dict(combo=dict(reach=R, excl=True), v_max=2.2868, d_max=0.7696, segs=set(), approved=[])

    def in_target(pose):
        if 'z' in pose:
            return None
        return all(abs(v) <= 70. + 1e-6 for v in pose.values())

    def depth_of(pa, pb, pose):
        if (pa, pb) == ('c', 'd'):
            raise RuntimeError('no depth')
        return 0.3
    approved = [dict(id='ARC-T1', joints=['a', 'b'], pair=frozenset(('g', 'h')), q1=[5., 15.], q2=[5., 15.], cap=3.0,
                     date='2026-09-28', by='u', why='w', evidence='')]
    rows = [('m', 'all_hi', {'a': 80., 'b': 0., 'c': 0.}, ('p', 'q', 30.0)),      # 区间外 → workspace
            ('m', 'all_lo', {'a': 50., 'b': 50., 'c': 50.}, ('p', 'q', 8.0)),      # 没到过 → retrain（带深度）
            ('m', 't1', {'a': 10., 'b': 10., 'c': 10.}, ('p', 'q', 1.0)),          # 到过、轻碰
            ('m', 't2', {'a': 10., 'b': 10., 'c': 10.}, ('c', 'd', 1.0)),          # 到过、深度算不出 → viol
            ('m', 't3', {'a': 10., 'b': 10., 'c': 10.}, ('g', 'h', 2.5)),          # 到过、超轻碰、盒内 ≤ 上限 → approved
            ('m', 't4', {'a': 10., 'b': 10., 'c': 10.}, ('g', 'h', 3.5)),          # 盒内超上限 → viol
            ('m', 't5', {'a': 10., 'b': 10., 'c': 10., 'z': 5.}, ('p', 'q', 1.0))]  # 目标区间分不出 → unknown
    cls, ret, light, viol, ws, appr = L6._classify_pose_rows(rows, env, pol, depth_of, in_target, approved)
    A.append(dict(claim='classes aligned with rows: workspace / retrain / light / viol(depth error) / approved / viol(over cap) / unknown',
                  ok=cls == ['workspace', 'retrain', 'light', 'viol', 'approved', 'viol', 'unknown']))
    A.append(dict(claim='retrain item carries mode/tag/pair/volume/depth and zero-free pose',
                  ok=len(ret) == 1 and ret[0]['mode'] == 'm' and ret[0]['tag'] == 'all_lo' and ret[0]['pair'] == 'p×q'
                  and ret[0]['vol_mm3'] == 8.0 and ret[0]['depth_mm'] == 0.3 and ret[0]['pose'] == {'a': 50., 'b': 50., 'c': 50.}))
    A.append(dict(claim='light row has depth; approved row names cap/id; both viol whys explain (depth error / over cap)',
                  ok=len(light) == 1 and light[0]['depth_mm'] == 0.3 and len(appr) == 1 and appr[0]['id'] == 'ARC-T1' and appr[0]['cap'] == 3.0
                  and len(viol) == 2 and any('算不出' in r['why'] for r in viol) and any('超过批准上限' in r['why'] for r in viol)))
    A.append(dict(claim='workspace row kept with volume; zero-volume-free pose', ok=len(ws) == 1 and ws[0]['vol_mm3'] == 30.0 and ws[0]['pose'] == {'a': 80.}))
    A.append(dict(claim='retrain item depth error → string, never green by omission',
                  ok=L6._classify_pose_rows([('m', 'x', {'a': 50., 'b': 50., 'c': 50.}, ('c', 'd', 8.0))], env, pol, depth_of, in_target, [])[1][0]['depth_mm'].startswith('算不出')))
    A.append(dict(claim='_row_block_flag: combo/pose viol → True, other classes → False, unknown → None, no class → in_target',
                  ok=L6._row_block_flag(dict(combo_class='viol', in_target=False)) is True
                  and L6._row_block_flag(dict(pose_class='viol', in_target=False)) is True
                  and L6._row_block_flag(dict(pose_class='retrain', in_target=True)) is False
                  and L6._row_block_flag(dict(pose_class='light', in_target=True)) is False
                  and L6._row_block_flag(dict(pose_class='workspace', in_target=True)) is False
                  and L6._row_block_flag(dict(pose_class='approved', in_target=True)) is False
                  and L6._row_block_flag(dict(pose_class='unknown', in_target=True)) is None
                  and L6._row_block_flag(dict(in_target=True)) is True and L6._row_block_flag(dict(in_target=False)) is False))
    pol_real = L6._load_real_pose_policy()
    A.append(dict(claim='real policy file: policy_extreme_poses.classify == pair_combination (v6)',
                  ok=(pol_real.get('pep') or {}).get('classify') == 'pair_combination'))
    import yaml
    from unittest.mock import patch
    raw = yaml.safe_load(L6._REAL_POSE_POLICY.read_text(encoding='utf-8'))
    with tempfile.TemporaryDirectory(prefix='duck_pep_') as t2:
        p2 = Path(t2) / 'p.yaml'
        r_no = dict(raw); r_no.pop('policy_extreme_poses', None)
        p2.write_text(yaml.safe_dump(r_no, allow_unicode=True), encoding='utf-8')
        with patch.object(L6, '_REAL_POSE_POLICY', p2):
            A.append(dict(claim='policy without policy_extreme_poses section → pep None (layer judges unknown)', ok=L6._load_real_pose_policy().get('pep') is None))
        r_bad = dict(raw); r_bad['policy_extreme_poses'] = dict(raw['policy_extreme_poses'], classify='target_range_only')
        p2.write_text(yaml.safe_dump(r_bad, allow_unicode=True), encoding='utf-8')
        with patch.object(L6, '_REAL_POSE_POLICY', p2):
            A.append(dict(claim='policy_extreme_poses.classify other than pair_combination rejected', ok=_raises(L6._load_real_pose_policy)))
    out = result(NAME, '全轴到过判定；区间外 WARN、没到过 → 重训约束、到过 → 轻碰 / 批准 / 违规；pose_class 同权；口径段缺 None / 错拒',
                 all(a['ok'] for a in A), '; '.join(f"{a['claim']}={a['ok']}" for a in A), [])
    out['assertions'] = A
    return out


if __name__ == '__main__':
    r = run(); print(json.dumps(r, ensure_ascii=False, indent=1)); sys.exit(0 if r['passed'] else 1)
