#!/usr/bin/env python3
"""L7 反例共用夹具（2026-09-13 审计第二批 F-L7-3/4/5/6）—— 不是 n_*.py，runner 不会当反例跑它。

真数据下（2026-09-13）L7 的整机判据（`joint:*:static_torque` / `_stability:com_in_support_home` /
`_load:head_mass_budget`）因**质量清单有缺口**一律 unknown（F-L7-4：15 类元件无质量、轴承跨 body、
electronics 没有 placed 实体……）—— 这是要的结果，不是要修掉的红。但反例要证明"清单完整时判据会绿、
坏样本会红"，对照就得先把缺口堵上。堵法**只删不编**（元规则 4 反向：不许给实物未知的数填估值）：

  · 没有 placed 实体也没有目录质量的元件（主板/IMU/摄像头/屏/麦/小板/降压/总线板/插头/线/开关/绑带/耗材）
    一律 qty=0（= 不装），不发明质量；
  · 轴承里没有目录质量的两种（bearing_6702zz / bearing_22x16x4）qty=0，并把它们的 placed 实体从替身目录拿掉；
  · original_prints：mass_g 是字符串（components.yaml 自己写了"故意不给标量"）、qty 4 而 placed/ 只有 3 个 orig_*
    → qty=0 + 拿掉 orig_* 实体；
  · 有目录质量的 bearing_6704zz / bearing_6700zz：housed_by 收成**单一宿主件**（真数据跨两个 body、层不猜，
    见 components.yaml:bearing_6704zz.mass_g.why_this_cell_stays_red）；
  · 所有还在册（qty>0）的元件都要有 claim 规则 —— 真 components.yaml 已给；夹具不改规则。

**这些都是夹具，不是对真数据的主张。** 反例文件头必须写明用了本夹具。
"""
from __future__ import annotations
import os
from copy import deepcopy
from pathlib import Path
import sys

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402

TMP = core.ROOT / "tools/gate/out/_neg_tmp"

# 没有 placed 实体、也没有可用质量 → 夹具里当"不装"（qty=0）。这些 id 只在夹具里出现，层里没有。
NOT_INSTALLED_IN_FIXTURE = ("sbc_radxa_zero3w", "imu_icm42688", "camera_csi", "display_lcd", "mic_inmp441",
                            "sensor_small_boards", "buck_12v_5v", "bus_adapter", "connector_xt30", "cable_ph20",
                            "strap_velcro", "filament",
                            # hr41 落盘 2026-09-25（主设计 Lane C）：09-13 之后 components.yaml 新增、无 claim 也无质量的四条（hr28–hr38 线材/一分二）
                            #   —— 夹具没跟上，完整清单对照里它们各报 component_mass + component_body 红（n_l7_* 自 hr41b 起 5 个失败的根因之一）
                            "cable_ph20_ext", "cable_qt_dupont_m", "cable_dupont_ff",
                            # hr44reg 2026-09-25：components.yaml:bus_splitter 改名 splitter_ph20_1to2（同一实物 ×5，仍无 claim、质量 assumed 2.33 g 不认领）
                            #   —— 夹具跟着换 id，否则完整清单对照里它报 component_body 红（扎带 / 缠绕管进顶层 consumables，不在 components 列表，夹具不用管）
                            "splitter_ph20_1to2")
# was_until_2026_09_25_hr44reg: 上面元组末项是 "bus_splitter"
# was_until_2026_09_25_hr43e: 上面元组末尾还有一项 "tape_foam_1mm"（注：2026-09-25 主设计：hr43c 新增 tape_foam_1mm（泡棉双面胶耗材，无 claim 无质量）→ 同上归"不装"；hr43c 全链 5 个 n_l7_* 失败根因）
# hr43e（2026-09-25）：tape_foam_1mm 已从 components.yaml 撤成 bus_adapter 条目下的 note（L7 不认 claim.by='none'），夹具同步删掉这一项
# hr38：speaker / amp_max98357a 现在有 placed 实体（zz_speaker / zz_amp）但都还没称重 → 归到 DROP_WITH_ENTITIES；
#   original_prints 的三个 orig_* 实体在 hr38 已全部退出 placed（分别由 H04/H05/J01 取代），这条只剩"没建模的 TPU 软嘴"，没有实体可拿掉。
# 有 placed 实体但没有质量（或质量是字符串）→ qty=0 并把实体从替身目录拿掉
DROP_WITH_ENTITIES = {"bearing_6702zz": ("bearing_left_hip_yaw", "bearing_right_hip_yaw"),
                      "bearing_22x16x4": ("bearing_head_roll_B",),
                      "original_prints": (),
                      "speaker": ("zz_speaker",),
                      "amp_max98357a": ("zz_amp",),
                      # hr52 2026-09-29：ToF 雷达 tof_vl53l5cx 有 placed 实体 zz_tof、mass_g null（到货再称）→ 同 speaker/amp 归这里
                      #   （hr52 登记快车道 12 个 n_l6/n_l7 红的根因之一；进程内补上后 12 个全过，见 hr52_work/reg/neg_rootcause.py）
                      "tof_vl53l5cx": ("zz_tof",)}
# 有质量的轴承收成单一宿主件（夹具假设：外圈座那一侧）
# hr41 落盘 2026-09-25（Lane C）：bearing_6703zz（09-17 hr12 恢复的髋横滚/髋俯仰 ×4，4.2 g）housed_by 跨 L01/L02/L10/L03/L13 五件多个 body，
#   层照实定不出 body（component_body 红）—— 夹具同 6704 口径收成单一宿主 L01
SINGLE_HOST = {"bearing_6704zz": ["L01"], "bearing_6700zz": ["L07"], "bearing_6703zz": ["L01"]}
# hr41 落盘 2026-09-25（Lane C）：servo_s288 qty 15（hr38 加的嘴舵机 ID 14）但 claim=mjcf_servo_geom 只认 MJCF body.servos 里有驱动的 14 颗，
#   嘴舵机 placed/servo__jaw_soft_jaw 认领不到 → component_qty 14 ≠ 15。夹具"只删不编"：少装这一颗（qty −1、替身目录拿掉该实体）
DROP_ONE_OF = {"servo_s288": ("servo__jaw_soft_jaw",)}


def stl_map():
    """与 gate.Ctx.stl 同规则：cad/duck_s288/<件号>_*.stl。"""
    out = {}
    for p in sorted(core.STL_DIR.glob("*.stl")):
        pid = p.stem.split("_", 1)[0]
        out.setdefault(pid, p)
    return out


def complete_inventory(base):
    """真数据 → 质量清单完整的对照数据（深拷贝）。返回 (data, 要从替身 placed/ 拿掉的实体名)。"""
    d = deepcopy(base)
    drop = []
    for c in d["components"]["components"]:
        cid = c.get("id")
        if cid in NOT_INSTALLED_IN_FIXTURE:
            c["qty"] = 0
        elif cid in DROP_WITH_ENTITIES:
            c["qty"] = 0
            drop += list(DROP_WITH_ENTITIES[cid])
        elif cid in SINGLE_HOST:
            c["housed_by"] = list(SINGLE_HOST[cid])
        if cid in DROP_ONE_OF and isinstance(c.get("qty"), int):     # hr41 落盘 2026-09-25（Lane C）：见 DROP_ONE_OF 注
            c["qty"] = c["qty"] - len(DROP_ONE_OF[cid])
            drop += list(DROP_ONE_OF[cid])
    return d, drop


def placed_stand_in(tag, drop=(), mutate=None):
    """替身 placed/：真实体软链接；drop 里的不链；mutate(stem, src_path, dst_path) 返回 True 表示自己写了文件。"""
    d = TMP / f"l7_fixture_{tag}_p{os.getpid()}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    for p in sorted(core.PLACED.glob("*.stl")):
        if p.stem in drop:
            continue
        if mutate is not None and mutate(p.stem, p, d / p.name):
            continue
        os.symlink(p, d / p.name)
    return d


def run_l7(data, placed_dir=None, mjcf=None):
    """经层的 run(ctx)；只猴子补丁 PLACED / _SIM_MJCF（README 反例包对 L6/L7 整机层的允许做法）。"""
    from _harness import FakeCtx
    import l7_mass as L7
    old_placed, old_mjcf = L7.PLACED, L7._SIM_MJCF
    try:
        if placed_dir is not None:
            L7.PLACED = placed_dir
        if mjcf is not None:
            L7._SIM_MJCF = mjcf
        return L7.run(FakeCtx(data, stl_map(), {}))
    finally:
        L7.PLACED, L7._SIM_MJCF = old_placed, old_mjcf


def control(tag="control"):
    """一次到位：完整清单数据 + 对应替身目录。"""
    base = core.load_data()
    data, drop = complete_inventory(base)
    return base, data, placed_stand_in(tag, drop)
