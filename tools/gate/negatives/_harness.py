#!/usr/bin/env python3
"""反例回归包的公共脚手架（元规则 8：Gate 每次先证明自己每个坏样本都能红）。

约束（本目录所有 n_*.py 都遵守）：
  · 只用**内存里现造的解析几何**（trimesh 布尔出小网格 → 写成临时 STL），不碰 cad/duck_s288 的整鸭件；
  · 判据一律取真实的 tools/gate/data/*.yaml（tolerances / frozen），只把 parts / features /
    fasteners / relations 换成这一条反例自己的最小清单 —— 否则"红"可能红在别的数据上；
  · 每个 < 5 秒；
  · **必须经过层的 `run(ctx)`**（不是只测辅助函数）——函数改名/不再被 run 调用时反例要跟着红。

结果契约（runner.py 校验，缺一项算失败）：
    run() -> {"name", "passed", "expect", "got", "detail",
              "expect_severity": "BLOCK"|"WARN",        # 坏样本应该红在哪一级
              "red": [record, ...]}                     # 坏样本上真正红了的判据（assert_red 给）
    或内核级反例：{"name", "passed", "expect", "got", "assertions": [{"claim", "ok"}, ...]}
  `red` 里每条必须 state==FAIL、severity==expect_severity、measured 非 None（= 抓到缺陷，不是"算不出来"）。
  需要验"坏声明必须 unknown"的反例，另给 `allow_unknown_red: "<为什么 measured 为空是设计>"`。
  这条契约是 2026-09-13 审计 F-反-1 的修法：以前只验 state，min_wall_geometric / support_reachable
  降成 WARN 之后反例照过，"翻回 BLOCK 也没人抓"已经发生过一次。
"""
from __future__ import annotations


import sys
import tempfile
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for p in (str(GATE), str(GATE / "layers")):
    if p not in sys.path:
        sys.path.insert(0, p)

import core  # noqa: E402


# ── 临时件 ─────────────────────────────────────────────────────────────────
# 必须落在仓库里：层会对 STL 路径做 relative_to(ROOT) 记进 inputs（元规则 6/7 的 hash 绑定）。
# 每个进程用自己的子目录（pid）—— 以前共用一个目录且一导入就清空，两个反例并行跑会互相删文件
# （2026-09-13 观察到 fg13 几何因此变 unknown）。导入时顺手清掉 1 小时前的旧子目录。
import os as _os, shutil as _sh, time as _time
_TMP_ROOT = core.ROOT / "tools/gate/out/_neg_tmp"
_TMP = _TMP_ROOT / f"p{_os.getpid()}"
_TMP.mkdir(parents=True, exist_ok=True)
for _old in _TMP_ROOT.iterdir():
    try:
        # 根目录下别的反例自己管理的文件/目录（n_l4_*、n_l7_mass_torque 等）不动 —— 并行时不能互删
        if _old.is_dir() and _old != _TMP and _time.time() - _old.stat().st_mtime > 3600:
            _sh.rmtree(_old, ignore_errors=True)
    except OSError:
        pass


def export_stl(mesh, name: str) -> Path:
    """把内存网格写成二进制 STL —— 元规则 1：层只认导出的文件。"""
    p = _TMP / f"{name}.stl"
    mesh.export(str(p), file_type="stl")
    return p


def box(sx, sy, sz, center=(0.0, 0.0, 0.0)):
    import trimesh
    m = trimesh.creation.box(extents=(sx, sy, sz))
    m.apply_translation(np.asarray(center, float))
    return m


def cyl(r, h, center=(0.0, 0.0, 0.0), axis="z", sections=64):
    import trimesh
    m = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    if axis == "x":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == "y":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    m.apply_translation(np.asarray(center, float))
    return m


# ── 假 ctx（接口与 gate.py:Ctx 一致的最小子集）──────────────────────────────
class FakeCtx:
    def __init__(self, data, stl_map: dict, placed_map: dict | None = None, only=None):
        self.data = data
        self.only = set(only) if only else None
        self.parts = [p["id"] for p in (data.get("parts", {}).get("parts") or [])]
        self._stl = {k: Path(v) for k, v in stl_map.items()}
        self._placed_index = {k: Path(v) for k, v in (placed_map or {}).items()}
        self._mesh = {}

    def stl(self, pid):
        return self._stl.get(pid)

    def placed(self, name):
        return self._placed_index.get(name)

    def placed_for(self, pid):
        rec = next((p for p in (self.data.get("parts", {}).get("parts") or []) if p["id"] == pid), None)
        cands = []
        if rec:
            inv = rec.get("inventory_id") or ""
            cands += [inv, inv.split("_", 1)[-1] if "_" in inv else inv]
        cands.append(pid)
        for c in cands:
            for stem in self._placed_index:
                if stem == c or stem.rstrip("_R") == c:
                    return self._placed_index[stem]
        return None

    def mesh(self, path, solid=False):
        k = (str(path), solid)
        if k not in self._mesh:
            import trimesh
            self._mesh[k] = trimesh.load(str(path), process=bool(solid))
        return self._mesh[k]

    def solid(self, path):
        return self.mesh(path, solid=True)

    def mjcf(self):
        """与 gate.Ctx.mjcf 一致：L6/L7 反例需要运动树时用真 kin（反例改的是几何/数据，不是运动学）。"""
        if not hasattr(self, "_mjcf") or self._mjcf is None:
            import sys as _s
            root = str(core.ROOT)
            if root not in _s.path:
                _s.path.insert(0, root)
            from duckstructure import kin
            self._mjcf = kin.load()
        return self._mjcf


def base_data(parts, features, fasteners=None, relations=None):
    """真实公差表 + 反例自己的最小清单。判据必须来自真数据，几何才是造的。"""
    d = core.load_data()
    d["parts"] = {"parts": parts}
    d["features"] = {"features": features, "frames": {"export_local": "反例：件自己的导出系"}}
    # hr41 落盘 2026-09-25（主设计 Lane C）：真 printability.yaml:no_support_zones.feature_zones 按**特征 id** 登记（现役 B03-F03..F06）；
    #   反例换成自己的最小特征清单后，这些 id 在清单里不存在 → no_support_zone_set 对每个件报 problems（"不存在的特征 id 属于谁说不清"），
    #   把与本反例无关的 L1 判据全拖成 unknown。这里只留清单里有的特征的条目（反例自己要测 feature_zones 的，在拿到 base_data 之后自行覆盖）。
    import copy as _copy
    nsz = ((d.get("printability") or {}).get("no_support_zones") or {})
    if isinstance(nsz.get("feature_zones"), dict):
        d["printability"] = _copy.deepcopy(d["printability"])
        ids = {f.get("id") for f in features}
        fz = d["printability"]["no_support_zones"]["feature_zones"]
        d["printability"]["no_support_zones"]["feature_zones"] = {k: v for k, v in fz.items() if k in ids}
    d["fasteners"] = {"fasteners": fasteners or [], "known_issues": [], "counts": {}}
    if relations is not None:
        d["relations"] = {"relations": relations, "counts": {"open": 0}}
    d["waivers"] = {"waivers": []}
    return d


# ── 结果查询 ───────────────────────────────────────────────────────────────
def findings(res, subject=None, check=None, check_prefix=None):
    out = []
    for f in res.findings:
        if subject is not None and f.subject != subject:
            continue
        if check is not None and f.check != check:
            continue
        if check_prefix is not None and not f.check.startswith(check_prefix):
            continue
        out.append(f)
    return out


def worst_state(fs):
    """一组 finding 的合成状态（与 core.build_scorecard 的格子取最差一致）。"""
    if not fs:
        return "ABSENT"
    rank = {"FAIL": 0, "STALE": 1, "NOT_RUN": 2, "WAIVED": 3, "PASS": 4}
    return min((f.state for f in fs), key=lambda s: rank.get(s, 9))


def summarize(fs, n=3):
    return " ; ".join(f"{f.check}={f.state}(sev={f.severity},n={f.evidence_n},measured={f.measured})"
                      for f in fs[:n]) or "（一条判据都没有 = 根本没查）"


def record(f) -> dict:
    """一条 finding 的可序列化摘要（runner 用它校验 severity / measured）。"""
    return {"check": f.check, "subject": f.subject, "state": f.state, "severity": f.severity,
            "measured": (None if f.measured is None else str(f.measured)[:80]),
            "evidence_n": int(f.evidence_n or 0)}


def assert_red(fs, severity="BLOCK", require_measured=True, check=None):
    """坏样本必须红：至少一条 finding state==FAIL、severity==期望、（默认）measured 非 None。
    返回 (ok, why, records)。records 就是要放进结果 `red` 字段的东西。
    `require_measured=True` 把"算不出来（unknown）"和"抓到缺陷"分开 —— unknown 也是 FAIL，
    但它证明不了判据会抓坏几何。"""
    fs = [f for f in fs if (check is None or f.check == check)]
    if not fs:
        return False, "一条 finding 都没有（根本没查）", []
    reds = [f for f in fs if f.state == "FAIL"]
    if not reds:
        return False, "没有 FAIL：" + summarize(fs), [record(f) for f in fs]
    bad_sev = [f for f in reds if f.severity != severity]
    if bad_sev:
        return False, (f"红了但严重级不是 {severity}：" + summarize(bad_sev)), [record(f) for f in reds]
    if require_measured:
        unk = [f for f in reds if f.measured is None]
        if unk:
            return False, "红了但是 unknown（measured 为空 = 算不出来，不是抓到）：" + summarize(unk), [record(f) for f in reds]
    return True, "", [record(f) for f in reds]


def assert_green(fs, check=None):
    """对照样本必须绿（PASS，且有证据）；ABSENT 也算不绿。返回 (ok, why)。"""
    fs = [f for f in fs if (check is None or f.check == check)]
    if not fs:
        return False, "对照样本一条 finding 都没有"
    bad = [f for f in fs if f.state != "PASS" or int(f.evidence_n or 0) == 0]
    if bad:
        return False, "对照样本不绿：" + summarize(bad)
    return True, ""


def result(name, expect, passed, got, red, expect_severity="BLOCK", detail="", allow_unknown_red=None):
    """按 runner 契约拼结果。"""
    out = {"name": name, "passed": bool(passed), "expect": expect, "got": got,
           "expect_severity": expect_severity, "red": list(red or []), "detail": detail}
    if allow_unknown_red:
        out["allow_unknown_red"] = str(allow_unknown_red)
    return out


def verdict(name, expect_red_check, fs, detail=""):
    st = worst_state(fs)
    return {"name": name, "expect_red_check": expect_red_check, "state": st,
            "red": st == "FAIL",
            "detail": (detail + " | " if detail else "") + summarize(fs)}


# ── L1 真 G-code 支撑落点桩（2026-09-13 C-1；取代旧的 slice_stub_observe 两遍跑）──────────────
def landing_stub(data, pid, stl, support_lines_local, down=(0, 0, -1), extra_argv=None, mutate_zone_data=None):
    """给 data["printability"]["slice_run"]["parts"][pid] 塞一条切片记录 + 由**合成 G-code** 算出的 support_landing。

    support_lines_local: [((x0,y0,z0),(x1,y1,z1)), …] —— export_local 里"切片器放了支撑挤出线"的位置（水平段）。
    走的是真实路径：线段按 slice_l1.main 同一变换（align_vectors(down→(0,0,-1)) + 平移到 (100,100,0)）转成打印坐标，
    写成 ;TYPE:Support material 的 G1 段，再交给 slice_l1.support_landing 解析 / 按 criteria 步长采样 / 逆变换 /
    对 l1_printable.no_support_zone_set(data) 逐区数点。桩只提供"支撑在哪"这一事实，不证明切片器会那么放；
    分级 / 阈值 / sha 核对全在层里。mutate_zone_data(data) 可在**建桩之后**改禁撑区声明（验 zones_sha256 不一致）。
    返回 (data, landing)。"""
    import sys as _s
    for _p in (str(GATE / "slicing"), str(GATE / "layers")):
        if _p not in _s.path:
            _s.path.insert(0, _p)
    import trimesh, slice_l1, l1_printable as L1
    P = data["printability"]
    step = float(P["criteria"]["support_landing_sample_step_mm"]["v"])
    m = trimesh.load_mesh(str(stl), process=False)
    M = trimesh.geometry.align_vectors(np.asarray(down, float), [0, 0, -1])
    o = m.copy(); o.apply_transform(M)
    shift = np.array([100.0, 100.0, 0.0]) - np.array([*o.bounds.mean(0)[:2], o.bounds[0, 2]])
    lines = ["M82", "G92 E0"]
    e = 0.0
    for k, (a, b) in enumerate(support_lines_local):
        pa = M[:3, :3] @ np.asarray(a, float) + shift
        pb = M[:3, :3] @ np.asarray(b, float) + shift
        lines += [";LAYER_CHANGE", f";Z:{pa[2]:.3f}", ";HEIGHT:0.2", f"G1 Z{pa[2]:.3f} F600",
                  ";TYPE:Support material", f"G1 X{pa[0]:.3f} Y{pa[1]:.3f} F3000"]
        e += 0.05 * max(1.0, float(np.hypot(*(pb - pa)[:2])))
        lines.append(f"G1 X{pb[0]:.3f} Y{pb[1]:.3f} E{e:.4f}")
    text = "\n".join(lines) + "\n"
    feats = (data.get("features") or {}).get("features") or []
    zs = L1.no_support_zone_set(P, feats, pid)
    landing = slice_l1.support_landing(text, M, shift, zs, step)
    srun = P.setdefault("slice_run", {})
    srun.setdefault("slicer", "反例桩（合成 G-code，非 PrusaSlicer 输出）")
    srun.setdefault("parts", {})[pid] = {
        "source_sha256": core.sha256_file(stl), "returncode": 0, "warnings": [],
        "down_normal_export_local": list(down),
        "support_volume_mm3": round(landing["support_path_mm"] * 0.4 * 0.2, 3),
        "slicer_argv_extra": list(extra_argv or []),
        "orient_transform": {"align_vectors_M": np.asarray(M).tolist(), "shift": shift.tolist()},
        "support_landing": landing,
        "_note": "反例桩：support_landing 由合成 G-code 经 slice_l1.support_landing 真算，不是真切片"}
    if mutate_zone_data is not None:
        mutate_zone_data(data)
    return data, landing
