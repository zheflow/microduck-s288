#!/usr/bin/env python3
"""反例（09-13 F-L7-2）—— 15 个 body 只有 1 个 <inertial> 给了惯量张量，`inertia_tensor_parsed` 必须红。

坑的形状：l7_mass._mjcf_inertial 末尾 `state=PASS if n_I else FAIL` —— 只要**任何一个** body
解析出张量整条就绿，而 criterion 写的是"MJCF **每个** <inertial> 必须给出惯量张量"。
14 个只写 mass/pos 的 body 由 MuJoCo 从 <geom> 反算，本层明说"不猜"，却照样放行。

正确判据：n_I == len(out)（每个 body 都要有）。
对照：15 个 body 全部带 diaginertia → 绿。

_mjcf_inertial 直接读 Path(kin.MD)/robot_walk.xml，所以本反例把一份最小 MJCF 写到
tools/gate/out/_neg_tmp/ 并把 kin.MD 临时指过去；finding 仍然经 res.add 发出（不是另写测试桩）。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import _TMP, assert_green, assert_red, findings, result  # noqa: E402
import core  # noqa: E402

NAME = "15 个 body 只有 1 个有惯量张量：inertia_tensor_parsed 必须红，全有必须绿"
EXPECT = "L7/_mjcf:inertia_tensor_parsed FAIL(BLOCK, measured='1/15')；对照 15/15 PASS"
N_BODY = 15


def _mjcf(n_with_tensor: int) -> str:
    """嵌套 15 个 body；前 n_with_tensor 个带 diaginertia，其余只有 mass/pos。joint 全是 axis=(0 0 1)。"""
    lines = ['<mujoco model="neg_l7_inertia"><worldbody>']
    for i in range(N_BODY):
        attrs = 'mass="0.01" pos="0 0 0.01"'
        if i < n_with_tensor:
            attrs += ' diaginertia="1e-6 1e-6 1e-6"'
        lines.append(f'<body name="b{i}" pos="0 0 0.02"><inertial {attrs}/>'
                     + (f'<joint name="j{i}" axis="0 0 1"/>' if i else ""))
    lines.append("</body>" * N_BODY + "</worldbody></mujoco>")
    return "\n".join(lines)


def _observe(n_with_tensor: int):
    root = str(core.ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)
    from duckstructure import kin
    import l7_mass
    d = _TMP / f"l7_inertia_{n_with_tensor}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "robot_walk.xml").write_text(_mjcf(n_with_tensor), encoding="utf-8")
    old = kin.MD
    kin.MD = str(d)
    try:
        res = core.LayerResult(7, "质量与力")
        out = l7_mass._mjcf_inertial(res)
    finally:
        kin.MD = old
    fs = findings(res, subject="_mjcf", check="inertia_tensor_parsed")
    return fs, len(out)


def run() -> dict:
    bad_fs, n_bad = _observe(1)
    good_fs, n_good = _observe(N_BODY)
    ok_bad, why_bad, red = assert_red(bad_fs, severity="BLOCK")
    ok_good, why_good = assert_green(good_fs)
    ok = ok_bad and ok_good and n_bad == N_BODY and n_good == N_BODY
    got = (f"1/{N_BODY} 样本 {bad_fs[0].state if bad_fs else 'ABSENT'}(measured={bad_fs[0].measured if bad_fs else None}) / "
           f"{N_BODY}/{N_BODY} 对照 {good_fs[0].state if good_fs else 'ABSENT'}(measured={good_fs[0].measured if good_fs else None})")
    return result(NAME, EXPECT, ok, got, red, expect_severity="BLOCK",
                  detail=" | ".join(x for x in (why_bad, why_good) if x)
                         or f"解析到 {n_bad}/{n_good} 个 body；finding 经 res.add 发出")


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
