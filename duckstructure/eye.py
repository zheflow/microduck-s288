"""hr39c 圆眼装饰（用户 2026-09-23：「想要圆眼装饰（黑眼珠 + 白外圈 + 高光，方摄像头嵌在中间）」）。

三件**独立打印、每件单色**，装在 H04 脸板前面（x = 脸板外面 93.47），圆心 = 摄像头光轴（head.H04_CAM_C = (y 0, z 257.94)）：
  E01 白外圈（白）：外 Ø EYE_D（默认 32，参数）、内 Ø PUPIL_D + 0.4，厚 1.6；背面 3 根 Ø1.8×1.0 定位销插进 H04 的 3 个 Ø2.0 通孔（head.build_face_plate 里开），CA 胶/UV 胶粘。
  E02 黑眼珠（黑）：Ø PUPIL_D、厚 2.0，中间 13.6 方孔套在摄像头镜座（13.2 方，穿出脸板 6.8）上 —— 镜座就是瞳孔里的"方眼"；背面贴脸板，胶粘。
  E03 高光点（白）：Ø HL_D、厚 1.2，按进黑眼珠左上（观者看）Ø HL_D+0.2 × 0.8 的浅坑，凸出 0.4，胶粘。
不焊接、不用螺丝（螺丝头会破坏眼睛外观）。尺寸全是外观参数，没有实物依据可言；EYE_D 可改（改完重跑 build，H04 销孔跟着走）。
打印朝向：三件都平放（E01 前面朝下，销朝上；E02 / E03 背面朝下），无悬垂。
━━ hr41（2026-09-24，摄像头贴板：去 H04 短立柱、PCB 贴面板内面，镜头前端前移 5.9 到 x 106.97）━━
  * E02 黑眼珠改 Ø24 圆筒：从脸板外面 93.47 一直到镜头前端 − 0.3（= 106.67，E02_FRONT_BACKOFF），中间照旧 13.6 方孔套镜座（定心）。
    前沿在镜头前端平面之后 → 任何 ≤180° 视场角都挡不到（视场角没查到，0.3 退量是 assumed 余量）。整只眼比 hr39c 多鼓 ≈11.2（2.0 → 13.2）。
  * E03 高光坑加深到 1.0，E03 只凸出 0.2（顶面 106.87 仍在镜头前端 106.97 之后）。
  * E01 白外圈：3 根定位销取消（H04 上对应 3 个 Ø2 孔全落在 PCB 投影里，和 0.8 浅坑冲突，见 head.H04_CAM_*）→ 内圈 Ø24.4 套在 E02 圆筒外定心，胶粘在脸面。
"""
import math, numpy as np
from .s288 import cyl, union, diff
from .lib import wbox

EYE_D = 32.0                    # 白外圈外径（用户："外径默认 Ø32 做成参数"）
PUPIL_D = 24.0                  # 黑眼珠外径（外圈宽 (32−24.4)/2 = 3.8）
RING_T, PUPIL_T, HL_T = 1.6, 2.0, 1.2
SQUARE = 13.6                   # 镜座方孔（= H04 方孔 13.6：镜座 13.2 + 单边 0.2）
HL_D, HL_AT = 3.4, (-9.0, 4.0)  # 高光点 Ø / 位置（相对圆心的 (y, z)；观者看左上 = 世界 −y、+z）
PIN_D, PIN_L, PIN_R, PIN_ANG = 1.8, 1.0, 14.2, (90.0, 210.0, 330.0)   # 白外圈背面定位销（落在外圈 r 12.2..16 中间）
HOLE_D = 2.0                    # H04 上的销孔（通孔）—— hr41 起 H04 不再开（h04_pin_holes 只留作留痕）
E01_PINS = False                # hr41：E01 不要销（见文件头 hr41 段）
E02_FRONT_BACKOFF = 0.3         # hr41：E02 前沿比镜头前端退 0.3（assumed）
E03_PROUD = 0.2                 # hr41：E03 凸出 E02 前面 0.2（原 0.4）


def lens_front_x():
    from .electronics import CAM_PCB_X, CAM_BACK_TO_LENS
    return CAM_PCB_X[0] + CAM_BACK_TO_LENS                      # 106.97（PCB 背面 90.57 + 16.4 measured）


def pupil_x():
    """E02 圆筒 x 范围（后面贴脸面 93.47，前沿 = 镜头前端 − 0.3）"""
    x0, _, _ = _center()
    return x0, lens_front_x() - E02_FRONT_BACKOFF


def _center():
    from .head import H04_CAM_C, H04_PLATE_IN
    return H04_PLATE_IN + 1.3, H04_CAM_C[0], H04_CAM_C[1]      # (x 前面, y, z)


def _pins_yz():
    _, yc, zc = _center()
    return [(yc + PIN_R * math.cos(math.radians(a)), zc + PIN_R * math.sin(math.radians(a))) for a in PIN_ANG]


def build_eye_white():
    x0, yc, zc = _center()
    ring = diff(cyl(EYE_D, RING_T, (x0 + RING_T / 2, yc, zc), axis="x", sections=128), cyl(PUPIL_D + 0.4, RING_T + 1.0, (x0 + RING_T / 2, yc, zc), axis="x", sections=128))
    if not E01_PINS: return ring
    pins = [cyl(PIN_D, PIN_L + 0.02, (x0 - PIN_L / 2 + 0.01, y, z), axis="x") for (y, z) in _pins_yz()]
    return union(ring, *pins)


def build_eye_pupil():
    """hr41：Ø24 圆筒 93.47..106.67（改前 = Ø24 × 2.0 圆片，见 build_eye_pupil_was_until_2026_09_24_hr41）"""
    _, yc, zc = _center(); xa, xb = pupil_x()
    tube = cyl(PUPIL_D, xb - xa, ((xa + xb) / 2, yc, zc), axis="x", sections=128)
    sq = wbox((xa - 1.0, yc - SQUARE / 2, zc - SQUARE / 2), (xb + 1.0, yc + SQUARE / 2, zc + SQUARE / 2))
    dep = HL_T - E03_PROUD
    pocket = cyl(HL_D + 0.2, dep + 0.01, (xb - dep / 2 + 0.005, yc + HL_AT[0], zc + HL_AT[1]), axis="x")
    return diff(tube, sq, pocket)


def build_eye_highlight():
    _, yc, zc = _center(); _, xb = pupil_x()
    return cyl(HL_D, HL_T, (xb + E03_PROUD - HL_T / 2, yc + HL_AT[0], zc + HL_AT[1]), axis="x", sections=48)


def build_eye_pupil_was_until_2026_09_24_hr41():
    x0, yc, zc = _center()
    disk = cyl(PUPIL_D, PUPIL_T, (x0 + PUPIL_T / 2, yc, zc), axis="x", sections=128)
    sq = wbox((x0 - 1.0, yc - SQUARE / 2, zc - SQUARE / 2), (x0 + PUPIL_T + 1.0, yc + SQUARE / 2, zc + SQUARE / 2))
    pocket = cyl(HL_D + 0.2, 0.8 + 0.01, (x0 + PUPIL_T - 0.4 + 0.005, yc + HL_AT[0], zc + HL_AT[1]), axis="x")
    return diff(disk, sq, pocket)


def h04_pin_holes():
    """H04 上给白外圈定位销的 3 个 Ø2.0 通孔"""
    from .head import H04_PLATE_IN
    return [cyl(HOLE_D, 4.0, (H04_PLATE_IN + 0.65, y, z), axis="x") for (y, z) in _pins_yz()]


PARTS = {"E01": ("E01_eye_white.stl", "eye_white", build_eye_white),
         "E02": ("E02_eye_pupil.stl", "eye_pupil", build_eye_pupil),
         "E03": ("E03_eye_highlight.stl", "eye_highlight", build_eye_highlight)}
