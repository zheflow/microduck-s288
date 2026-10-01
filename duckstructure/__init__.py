"""duckstructure —— S288 版 Microduck 整鸭参数化 CAD，按身体部位组织的包（原 tools/cad/duck.py 拆分而来，几何一字未改）。

    ./.venv/bin/python -m duckstructure.build      # 在仓库根目录跑：导出 17 STL 到 cad/duck_s288/ + 全部检查

模块：s288（舵机参数/原语）→ kin（MJCF 运动学）→ lib（通用原语、参数表 P、全局常量）→ legs / neck → trunk / head → checks → build。
本包对外把各子模块的公开名字全部再导出（`import duckstructure as D` 后 D.sfw / D.P / D.build_trunk … 都可用），
tools/cad/{assembly_audit,mechanical_audit}.py 就是拿这个包对象当"duck 模块"用的。"""
import os as _os, sys as _sys
# tools/cad/ 里的检查脚本（assembly_audit / mechanical_audit / ankle_split / asmcheck）仍按顶层模块名被 import（原 duck.py 把自己所在目录塞进了 sys.path）
_TOOLS_CAD = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "tools", "cad")
if _TOOLS_CAD not in _sys.path: _sys.path.insert(0, _TOOLS_CAD)

from . import s288, kin
from .lib import *        # noqa: F401,F403  参数表 P、常量、几何原语、B/ORDER/TW/sfw/...
from .legs import *       # noqa: F401,F403
from .neck import *       # noqa: F401,F403
from .trunk import *      # noqa: F401,F403
from .head import *       # noqa: F401,F403
from .checks import *     # noqa: F401,F403
from .bearing_rebuild import build_hip_roll_sleeve
from .legs import build_yaw2roll, build_hip, build_upper_leg, build_lower_leg, build_ankle_parts, build_ankle_foot, build_ankle_rear_arm
from .trunk import build_trunk, build_trunk_shell, build_battery_door, battery_bay, battery_bay_cuts
from .neck import build_neck, build_neck_pitch, build_yrm, build_head_journal
from .head import build_head_bracket, build_head_clamp, build_head_bottom_shell
from .hip_pitch_bearing_rebuild import build_hip_pitch_sleeve
from .head_bearing_rebuild import build_head_roll_adapter, build_head_yaw_adapter, build_head_bearing_cap, build_head_yaw_cap

# 件号 → (STL 文件名, 导出用的 MJCF 连杆, build 函数)。顺序 = build.py 的导出顺序。18 件（09-12 ④ 加 N04）。
# L05/L06 出自同一次 build_ankle_foot()（返回 (主脚, 鞋底)）；T02/T03 要传 (side, trunk 网格)。
PARTS = {
    "L01": ("L01_yaw2roll.stl",          "yaw2roll",        build_yaw2roll),
    "L02": ("L02_hip.stl",               "hip_l",           build_hip),
    "L03": ("L03_upper_leg.stl",         "upper_leg_left",  build_upper_leg),
    "L04": ("L04_lower_leg.stl",         "leg",             build_lower_leg),
    "L05": ("L05_ankle_foot.stl",        "ankle_left",      build_ankle_foot),
    "L06": ("L06_sole_TPU.stl",          "ankle_left",      build_ankle_foot),
    "L07": ("L07_ankle_rear_arm.stl",    "ankle_left",      build_ankle_rear_arm),
    "L10": ("L10_hip_roll_sleeve.stl",   "hip_l",           build_hip_roll_sleeve),
    "L13": ("L13_hip_pitch_sleeve.stl", "hip_l", build_hip_pitch_sleeve),
    "T01": ("T01_trunk.stl",             "trunk_base",      build_trunk),
    "B01": ("B01_battery_door.stl",      "trunk_base",      build_battery_door),
    "T02": ("T02_shell_L.stl",           "trunk_base",      build_trunk_shell),
    "T03": ("T03_shell_R.stl",           "trunk_base",      build_trunk_shell),
    "N01": ("N01_neck.stl",              "neck",            build_neck),
    "N02": ("N02_neck_pitch.stl",        "neck_pitch",      build_neck_pitch),
    "N03": ("N03_yaw_roll.stl",          "yaw_roll_motion", build_yrm),
    "N04": ("N04_head_journal.stl",      "yaw_roll_motion", build_head_journal),   # 头横滚 B 端轴颈销（09-12 ④）
    "N05": ("N05_head_roll_adapter.stl", "yaw_roll_motion", build_head_roll_adapter),
    "N06": ("N06_head_yaw_adapter.stl", "neck_pitch", build_head_yaw_adapter),
    "N07": ("N07_head_bearing_cap.stl", "yaw_roll_motion", build_head_bearing_cap),
    "N08": ("N08_head_yaw_cap.stl", "yaw_roll_motion", build_head_yaw_cap),
    "H01": ("H01_head_bracket.stl",      "jaw_soft",        build_head_bracket),
    "H02": ("H02_head_clamp.stl",        "jaw_soft",        build_head_clamp),
    "H03": ("H03_head_bottom_shell.stl", "jaw_soft",        build_head_bottom_shell),
}

def build_all():
    """按 build.py 主流程相同的顺序建全部 17 件（世界坐标、零位姿），返回 {件号: 网格}。只建不导出、不检查。
    @memo 的件（L02/L03/L04/踝、N01/N02）同一进程内只算一次；T01 只建一次并复用给 T02/T03。"""
    out = {}
    out["L01"] = build_yaw2roll(); out["L02"] = build_hip(); out["L03"] = build_upper_leg(); out["L04"] = build_lower_leg()
    out["L05"], out["L06"] = build_ankle_foot(); out["L07"] = build_ankle_rear_arm()
    out["L10"]=build_hip_roll_sleeve()   # hr08：髋横滚只剩轴套（整体座在 L01 里）
    out["L13"]=build_hip_pitch_sleeve()   # hr07：髋俯仰只剩轴套（整体座在 L03 里）
    tr = build_trunk(); out["T01"] = tr; out["B01"] = build_battery_door()
    out["T02"] = build_trunk_shell(1, tr); out["T03"] = build_trunk_shell(-1, tr)
    out["N01"] = build_neck(); out["N02"] = build_neck_pitch(); out["N03"] = build_yrm(); out["N04"] = build_head_journal()
    out["H01"] = build_head_bracket(); out["H02"] = build_head_clamp(); out["H03"] = build_head_bottom_shell()
    for name in ("N05", "N06", "N07", "N08"):
        out[name] = PARTS[name][2]()
    return out
