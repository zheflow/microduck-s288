"""BAM synchronization compares signed domains and both torque endpoints."""
from __future__ import annotations

import contextlib
import copy
import json
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))
import _l6_scene  # real Gate import paths; no CAD scene is constructed
import core
import l6_motion as L6
from _harness import findings, record, result

NAME = "BAM双向端点/标量取值域/坏参数必须红，正常同步仍绿"


def run():
    assertions, reds = [], []
    parameters = {k: v for k, v in json.loads(L6._BAM_RELEASE.read_text()).items()
                  if k in {f[0] for f in L6._BAM_FIELDS}}
    torque = parameters["max_torque"]
    defaults = {attr: (f"{-parameters[key]:.6g} {parameters[key]:.6g}" if attr == "forcerange"
                       else f"{parameters[key]:.6g}") for key, _, attr in L6._BAM_FIELDS}
    model_sides = ("gen", "train", "allcol")

    def xml(attrs):
        return ('<mujoco><default><default class="chosen_actuator">'
                f'<joint frictionloss="{attrs["frictionloss"]}" damping="{attrs["damping"]}" '
                f'armature="{attrs["armature"]}"/>'
                f'<position forcerange="{attrs["forcerange"]}"/>'
                '</default></default></mujoco>')

    with tempfile.TemporaryDirectory(prefix="duck_bam_sync_negative_") as temp:
        d = Path(temp)
        paths = {key: d / name for key, name in (("release", "release.json"), ("pkg", "pkg.json"),
                 ("gen", "robot.xml"), ("train", "train.xml"), ("allcol", "allcol.xml"))}

        def check(label, *, json_edits=None, xml_edits=None, raw_json=None, expected="FAIL"):
            js = {side: copy.deepcopy(parameters) for side in ("release", "pkg")}
            attrs = {side: dict(defaults) for side in model_sides}
            for side, edits in (json_edits or {}).items():
                js[side].update(edits)
            for side, edits in (xml_edits or {}).items():
                attrs[side].update(edits)
            for side in ("release", "pkg"):
                paths[side].write_text((raw_json or {}).get(side, json.dumps(js[side])))
            for side in model_sides:
                paths[side].write_text(xml(attrs[side]))
            with contextlib.ExitStack() as stack:
                for attr, side in (("_BAM_RELEASE", "release"), ("_BAM_PKG", "pkg"),
                                   ("_GEN_MJCF", "gen"), ("_GEN_TRAIN_MJCF", "train"), ("_GEN_ALLCOL_MJCF", "allcol")):
                    stack.enter_context(patch.object(L6, attr, paths[side]))
                r = core.LayerResult(6, "negative")
                L6._bam_params_sync(r)
            fs = findings(r, subject="_simmodel", check="bam_params_sync")
            if expected == "PASS":
                ok = len(fs) == 1 and fs[0].state == "PASS" and fs[0].measured == 0 and fs[0].evidence_n > 0
                ok = ok and all(L6._rel(p) in r.inputs for p in paths.values())
            elif expected == "UNKNOWN":
                ok = len(fs) == 1 and fs[0].state == "FAIL" and fs[0].measured is None
            else:
                ok = (len(fs) == 1 and fs[0].state == "FAIL" and fs[0].severity == "BLOCK"
                      and fs[0].measured is not None and fs[0].measured > 0 and fs[0].evidence_n > 0)
                if label.startswith("one_way_zero"):
                    reds.extend(record(f) for f in fs)
            assertions.append(dict(claim=label, ok=bool(ok), detail="; ".join(f.detail for f in fs) if not ok else ""))

        check("normal_signed_endpoints_and_six_significant_digits", expected="PASS")
        zero_params = {k: 0. for k in parameters if k != "max_torque"}
        zero_attrs = {a: "0" for _, _, a in L6._BAM_FIELDS if a != "forcerange"}
        check("zero_losses_and_armature_are_valid", json_edits={s: zero_params for s in ("release", "pkg")},
              xml_edits={s: zero_attrs for s in model_sides}, expected="PASS")

        ranges = {
            "one_way_zero_positive": f"0 {torque}", "one_way_zero_negative": f"{-torque} 0",
            "reversed_endpoints": f"{torque} {-torque}", "both_negative": f"{-torque} {-torque}",
            "both_positive": f"{torque} {torque}", "one_endpoint": str(torque),
            "extra_endpoint": f"{-torque} 0 {torque}", "empty_endpoints": "",
            "wrong_negative_strength": f"{-torque/2} {torque}",
            "wrong_positive_strength": f"{-torque} {torque/2}",
            "nan_left": f"nan {torque}", "nan_right": f"{-torque} nan",
            "infinite_right": f"{-torque} inf", "infinite_left": f"-inf {torque}",
            "nonnumeric_endpoint": f"no {torque}",
        }
        for side in model_sides:
            for label, value in ranges.items():
                check(f"{label}_{side}", xml_edits={side: {"forcerange": value}})
            for key, _, attr in L6._BAM_FIELDS:
                if attr == "forcerange":
                    continue
                for label, value in (("negative", str(-parameters[key])), ("nan", "nan"), ("inf", "inf"),
                                     ("minus_inf", "-inf"), ("multiple", "0 0"), ("nonnumeric", "invalid")):
                    check(f"{side}_{attr}_{label}", xml_edits={side: {attr: value}})

        for side in ("release", "pkg"):
            for key in parameters:
                for label, value in (("negative", -parameters[key]), ("nan", float("nan")),
                                     ("inf", float("inf")), ("minus_inf", -float("inf")),
                                     ("string", str(parameters[key])), ("boolean", True), ("list", [1.])):
                    check(f"{side}_{key}_{label}", json_edits={side: {key: value}})
            check(f"{side}_zero_torque", json_edits={side: {"max_torque": 0.}})
            check(f"{side}_malformed_json", raw_json={side: "{"}, expected="UNKNOWN")
            check(f"{side}_json_array", raw_json={side: "[]"}, expected="UNKNOWN")
            check(f"{side}_null_parameter", json_edits={side: {"friction_base": None}}, expected="UNKNOWN")
        # Matching copies must not legitimize a nonphysical parameter domain.
        for key in parameters:
            edit = {key: -parameters[key]}
            check(f"both_json_identically_negative_{key}", json_edits={"release": edit, "pkg": edit})

    out = result(NAME, "有序双向扭矩端点及非负标量；错误数值BLOCK；缺/不可解析参数unknown", all(a["ok"] for a in assertions),
                 f"{sum(a['ok'] for a in assertions)}/{len(assertions)} assertions", reds)
    out["assertions"] = assertions
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
