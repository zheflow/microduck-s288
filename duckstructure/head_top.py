"""H05 head_top_shell（hr38，2026-09-23）：原版 top_head_shell 的派生件 = 原版 − 通风槽（F01）− 喇叭声孔（F02）+ 喇叭卡座（F03）+ 功放卡座（F04）。
原版文件不改（frozen.yaml sha256 那份原封不动）；照 head.py:build_head_bottom_shell 的套路：拿 lib.orig() 当毛坯，只做减/加，收尾 keep_main + clean_print_topology。
原模型来自 Pollen Robotics Microduck，沿用原资产的 CC BY-SA-NC 许可。

坐标全是世界系零位（与 head.py 同）。打印朝向按原版（printability.yaml:CAL_top_head_shell down=[0,−1,0] 资产系 = 世界 +x 朝下，脸口贴床、层沿 −x 堆）：
  → 世界 −z 朝向的面 / ±y 面在打印里是竖直面，随便长；**世界 +x 朝向的面才是悬垂面**。所以本文件里从顶壳内面长出来的每一块料，
    +x 端都做成 45° 船头（_prow_bar：+x 面改成 x–y 平面里的 V，顶点外伸 = 半宽）或者 y 向宽度 ≤ 2 mm（首层 2 mm 的小悬垂，与 2 mm 槽的封顶桥一个量级）。
  → 通风槽/声孔：槽沿 x 长、y 向宽 2 —— 每层只是 2 mm 的缺口，末端封顶是 2 mm 桥；Ø2.2 的横孔也不用支撑。

安装位置（都是本轮量出来的，依据见各常量注释；hr38_顶壳与扬声器.md 有整表）：
  * 喇叭 Ø28×5 平放（轴沿 z）在**头顶前部** 中心 (50, 2.5)、x 36..64、z 264.5..269.5：不是用户先说的后脑 —— 后脑 x 12..40 那段被 N03/偏航舵机绕头横滚 ±30° 的扫掠体占到 z 266.5（y≈−10），
    零位体素扫描看不见这个；后脑 x −17..11 能放喇叭但功放就没地方了（带 8.5 高螺丝端子的功放板在 x 42 处只剩 0.5 mm）。前部：扫掠体 x ≥ 40 后 ≤ 245，
    Radxa 排针顶 262.5、H01 前立板顶 262，顶壳内面 271.1..273.8（横筋底 270.1）→ 喇叭卡爪底 263.3，六向都有料夹住。声孔阵列在喇叭正上方（同时给 Radxa 当出风口）。
  * 功放 MAX98357A 20×18 板平放在**后脑** H01 上桥（顶 249.5）之上 z 252.5..254.1，螺丝端子朝上。前提：**不焊直排针**（直针 + 14 mm 杜邦母壳在头顶哪里都放不下，
    见报告 §3），信号/电源 7 根线焊在排针焊盘上（卖家代焊或自己焊）。
"""
import math, numpy as np, trimesh
from .s288 import cyl, union, diff, inter
from .lib import orig, wbox, keep_main, clean_print_topology
from . import electronics as E
from . import head_scale as HS
# ━━ hr39c（2026-09-23 头等比放大每侧 +10）：毛坯换成 head_scale.scaled_orig("top_head_shell")（绕 P0 放大 s、壁厚保持）；
#    喇叭 / 功放卡座整组跟顶壳内面走（T_SPK / E.T_AMP = head_scale.D 在卡座上方内面那一点的位移），肩面高度照原规则重算（筋底 − 0.6）；
#    F01 出风槽让开喇叭前挡挪到 x 71.5..77.5；按 hr39_头内线束.md §10b 在口袋（x 52..66）正上方加 7 条出风槽（F05）；
#    转接板挪到 Radxa 前面 → 顶壳内面加两条压筋压住板顶边（F06）；嘴：新枢轴孔 + 插入槽 + J01 扫掠让位（jaw.shell_pivot_cuts / relief_sweep）。
T_SPK = (5.18, 0.56, 12.25)       # head_scale.D(50, 2.5, 271) = (5.181, 0.556, 12.268)；z 取让肩面 = 筋底 − 0.6（281.75）

# ── F01 通风槽：Radxa ZERO 3W 板 x 66..67.6、元件面朝 +x 到 70.6、z 236..266（electronics.SBC_*）；热空气沿板两面上升到顶壳内面 z≈273.5（外顶 275.2，壁 1.7，hr38 射线量）。
#    槽开在板正上方偏前 x 67..77（板背面 x 66 到脸板凹槽 x 80 之间；凹槽前留 3 mm 筋），y 与 H03 进风槽同位 ±16/±8/0（H03_VENT_YS），宽 2、x 长 10 → 5×20 = 100 mm²
#    > H03 进风 5×2×7 = 70 mm²（tub 槽底那 5 条）。筋 6 mm。不到 x 66 是给 F03 喇叭前挡（x ≤ 65.5）留 1.5 mm 壁；不到 x 78 是脸板凹槽。
VENT_X = (71.5, 77.5)                 # hr39c：原 67..77 → 喇叭跟顶壳前移 5.2 后前挡在 x 69.4..70.4，槽让到 71.5 起（仍在 Radxa CSI 面 x 67.6..70.6 前上方）
VENT_X_was_until_2026_09_23_hr39c = (67.0, 77.0)
VENT_Z = (276.0, 297.0)               # 刀 z（放大后内面 ≈ 286..288）
VENT_W = 2.0
VENT_YS = (-16.0, -8.0, 0.0, 8.0, 16.0)
# ── F03 喇叭卡座（electronics.SPK_*：Ø28×5 assumed，商品页标称）：
#    中心 SPK_C (50, 2.5)。顶壳内面在喇叭脚印 (36..64, 2.5±14) 上最低点：原版内面有一条横筋 x 52.2..53.1（1 宽、2 深，筋底 271.1 @y0 → 270.1 @|y|14，射线 0.25 步量），
#    肩面 z_seat = 269.5 留 0.6 到筋底（筋外的内面 ≥ 271.1，肩厚 ≥1.6）；喇叭 z 264.5..269.5；卡爪顶面 264.3（0.2 隙）、爪厚 1.0 → 爪底 263.3。
#    下方：Radxa 40 针塑座顶 262.5（隙 0.8）、H01 前立板顶 262（隙 1.3）、N03/偏航舵机 ±30° 横滚扫掠体（已含 grow 0.45）在 x ≥ 40 处 ≤ 245、在 x 36 (y −5) 263.7（喇叭底 264.5，隙 0.8）。
#    −y 导轨从 x 40.5 起（x 37..39、y −14..−16 处扫掠体到 265.3 > 爪底）；+y 侧扫掠体 ≤ 251.7，导轨也从 40.5 起（对称）。
#    卡法 = 斜插 + 单侧卡扣：−y 两个 1.0 爪是固定钩，+y 一个 0.8 爪带斜面，+y 导轨 1.2 厚、悬臂 ≈8.5 → 扣入挠 0.8 时 σ≈3·E·t·δ/(2L²)=3·2000·1.2·0.8/(2·72)≈40 MPa（PETG 屈服 ≈50，一次性扣入可接受；爪压住圆缘 0.6..0.7）。
SPK_C_was_until_2026_09_23_hr39c = (50.0, 2.5)
SPK_C = (50.0 + T_SPK[0], 2.5 + T_SPK[1])                 # hr38 复核：中心 (49,0) 时喇叭后缘 (36,−5) 与扫掠体（grow 0.45）交 3.6 mm³、(50,1) 时后缘 (38.5,−7.5) 离未膨胀扫掠体只 0.1、(50,2) 0.3 → 前移 1、偏 +y 2.5（扫掠体 −y 侧高、+y 侧低）；x 不能再前（前挡离 Radxa 板 0.8）
SPK_CLR = 0.2                       # 喇叭外径/厚度 单边隙
SPK_Z_SEAT = 281.75                 # 肩面（喇叭顶面贴这里）。hr39c：放大后喇叭脚印（r 14.5）内内面最低点 = 横筋底 282.35 @(57.7, 17.3) → −0.6（同 hr38 规则）；hr38 269.5
SPK_TAB_T = 1.0                     # 爪厚（z）
SPK_RAIL_X = (41.5 + T_SPK[0], 62.0 + T_SPK[0])           # 导轨 x（+x 端另加船头 0.75）
SPK_RAIL_T = {-1: 1.5, 1: 1.2}      # −y 固定导轨 / +y 卡扣导轨 厚
SPK_FLANGE_IN = 2.2                 # 肩（导轨内伸翻边）宽：盖住喇叭外缘 2.0（Ø28 → 内 Ø24 是常见振膜外径）
SPK_FLANGE_X = (40.0 + T_SPK[0], 60.0 + T_SPK[0])
# hr43d（加固：hr43_work/转接板安放研究.md §6.2、agent_brief_hr43d 第 5 条）：肩（±y 两条翻边）下表面抬 SPK_FOAM_RAISE 留泡棉环位 —— 1 mm 泡棉环（Ø28/Ø24）夹在肩与喇叭顶之间
#   0.5 + 喇叭对爪原有 0.2 隙 → 环压到 ≈0.7、喇叭被压在爪上（喇叭底实际比 SPK_Z_SEAT − 5 口径低 0.2，对转接板最坏 1.55，报告列）；另登记「外缘与导轨之间 2–3 点胶」。
#   喇叭本体 / 爪 / 导轨 / 前后挡 / 声孔都不动（SPK_Z_SEAT 仍 = 喇叭顶面）。肩面抬到 282.25 < 顶壳内面最低 282.35（横筋底，见 SPK_Z_SEAT 注）→ 肩实体含 EMBED 仍 ≥0.7。
SPK_FOAM_RAISE = 0.5
SPK_TABS = {-1: ((45.5 + T_SPK[0], 48.5 + T_SPK[0], 1.0), (51.5 + T_SPK[0], 54.5 + T_SPK[0], 1.0)), 1: ((48.5 + T_SPK[0], 51.5 + T_SPK[0], 0.8),)}   # (x0, x1, 内伸)：导轨是直条、只在 x=cx 与圆喇叭相切，爪必须贴着切点（|Δx|≤4.5 处圆缘 |Δy|≥13.26，爪尖 13.2 才压住 ≥1.0）；−y 爪不到 Radxa 排针 x 57.5
SPK_STOP_REAR = (34.3 + T_SPK[0], 1.5)         # 后挡：x0 与半宽；接触面 = 船头顶点 x = SPK_C[0] − r − CLR = 35.8
SPK_STOP_FRONT = (1.0, 1.0)         # 前挡：厚(x)、半宽(y)；接触面 x 64.2，体到 65.2（离 Radxa 板背面 66.0 留 0.8），船头放不下 → +x 面就是 2 mm 宽的小悬垂（与 2 mm 槽封顶桥同级），不船头
# ── F02 声孔：Ø2.2 六角阵列、间距 3.5（筋 1.3 ≥ 1.2 规则），中心半径 ≤ 10.7（孔边 ≤ 11.8 < 肩内缘 12.0），在喇叭正上方的壳顶（壁 1.7）。
SOUND_HOLE_D, SOUND_HOLE_PITCH, SOUND_HOLE_RMAX = 2.2, 3.5, 10.7
# ── F04 功放卡座（electronics.AMP_*）：板 x −10..10 × y −9..9 × z 252.5..254.1，H01 上桥顶 249.5（隙 3.0）、A 座环顶 251.1（x 6..10、|y|≤4）离板底焊点（0.5）0.9。
#    顶壳内面在 (−10, ±10) ≈261.5、(10, 0) ≈269.5 → 导轨 10..18 mm 高，随内面（_ceiling_solid）。板从下往上推入：−y 两爪 1.0 固定、+y 一爪 0.7 卡扣（+y 导轨 1.2 厚、L≈12 → σ≈18 MPa）。
#    翻边（肩）只压板的 ±y 边条 |y| 8.0..9.2，避开 +x 边排针焊盘（y ±8.9、x 7.4..10）与端子。−x 挡 = 立筋 x −13.7..−12.2 + 底脚伸到 −10.2（端子若在 −x 边、外沿与板齐，留 0.2）。
#    +x 挡 = 两条导轨末端的内伸唇 x 10.2..10.7（板边外）。
AMP_CLR = 0.2
AMP_RAIL_T = {-1: 1.5, 1: 1.2}
AMP_TAB_T = 1.0
# hr38：爪的 x 段要躲开**板底真有焊点**的两处 —— −y 边 7P 排针焊盘带（x ±8.9）与 +y 边螺丝端子脚（x ±3.8）。
#   所以 −y 两爪挪到板的两个 x 端（x −10..−9.1 / 9.1..10），+y 一爪挪到 x 4.5..7.5。
#   旧值 {-1: ((-8,-5,1.0),(3,6,1.0)), 1: ((-1.5,1.5,0.7),)} 是按「排针焊盘在 +x 边」的直排针版排的，弯针版作废。
AMP_TABS = {-1: ((-10.0, -9.1, 1.0), (9.1, 10.0, 1.0)), 1: ((4.5, 7.5, 0.7),)}
# hr38：肩（压板的翻边）改成**逐侧**给 —— 弯排针版的板 −y 边被弯针座骑住（y −11.6..−9.0，z 254.1..256.6），
#   那一侧压不下去；螺丝端子又挪到了 +y 边（x ±3.8），所以 +y 肩要躲开 x ±4.5。
#   结果：板的 −z 由三个爪托、+z 由 +y 两段肩压、±y 由两条导轨、−x 由立筋、+x 由两条导轨末端的唇；合壳后 H01 上桥（249.5）兜底。
AMP_FLANGE = {-1: (), 1: ((-9.0, -4.5), (4.5, 9.0))}      # 肩的 x 段（逐侧）
AMP_FLANGE_IN = 1.2                          # 肩内伸（从板边）= electronics.AMP_EDGE_BARE：只压板 ±y 长边 1.2 mm 的空边条
AMP_STOP_REAR = ((-13.7, -12.2), 1.5, 2.4)   # 立筋 x、半宽、底脚高（从爪底起）
AMP_LIP_FRONT = (0.5, (7.5, 9.2))            # +x 唇厚、|y| 范围
# hr38：功放改买**已焊弯排针**版（整鸭不焊接；直排针 + 14 杜邦壳在头里一个位都放不下，见 electronics.amp_boxes 注释）。
#   杜邦母壳从板的 −y 边平躺伸出 14（世界 y −23..−9，z 板顶 254.1..257.14）→ **−y 那条导轨必须在壳经过的那段开窗**。
#   打印朝向是世界 +x 朝下、层沿 −x 堆：导轨是沿 x 的长条，开窗 = 某几层没有这条导轨，**不是桥接**（窗两侧的导轨段各自逐层长上来）。
#   窗只切 z 253.9..257.6、y −12.5..−9.15 —— 不碰 −y 肩（y −9.2..−8.0，z ≥253.8）也不碰 −y 两个爪（z 250.8..251.8）。
AMP_SHELL_WIN_was_until_2026_09_25_hr43 = ((-9.4 + E.T_AMP[0], -12.5, 253.9 + E.T_AMP[2]), (9.4 + E.T_AMP[0], -9.15, 257.6 + E.T_AMP[2]))   # hr39c：跟功放平移 E.T_AMP
# hr39c（hr39_头内线束.md §4.9 / 硬伤⑤）：喇叭两根线从 +y 沿 −y 进功放 +y 边的螺丝端子（端子 x −3.8..3.8、线口 z ≈ 板顶 + 2.5），正对 +y 导轨（y 9.2..10.4）→
#   +y 导轨在两段肩（x ±4.5..±9）之间开窗 x −4.5..4.5 × z 板顶..板顶 + 4.4（hr38 板位 254.1..258.5，整组 + E.T_AMP）。板顶以下那截导轨（z 262.3..265.1）照旧挡板边；
#   +y 爪（x 4.5..7.5）与肩都在窗外。打印同 −y 窗：层沿 −x 堆，开窗 = 那几层没有导轨，不是桥接。
AMP_TERM_WIN_was_until_2026_09_25_hr43 = ((-4.5 + E.T_AMP[0], 9.15, 254.1 + E.T_AMP[2]), (4.5 + E.T_AMP[0], 12.5, 258.5 + E.T_AMP[2]))
EMBED = 0.6                                   # 新料顶面压进顶壳壁的深度（壁 1.7..2.4 → 余 ≥1.1）
# hr39c F05：口袋正上方出风槽（hr39_头内线束.md §10b 规格：x 53..65、槽宽 2、筋 1.5；y +20/+23.5/+27、−16/−19.5/−23/−26.5，避开喇叭导轨）
POCKET_VENT_X = (53.0, 65.0)
POCKET_VENT_YS = (20.0, 23.5, 27.0, -16.0, -19.5, -23.0, -26.5)
POCKET_VENT_Z = (276.0, 297.0)
# hr39c F06：转接板顶边压筋（板 x 72..81 × y −24..16 × 顶 265.4，head.H03_ADP_*）：两条沿 x 的 2 宽条（y ±12，躲开 F01 槽 y 0/±8/±16），
#   底面 = 板顶 + 0.3，+x 端船头；从内面长下来 ≈21 mm（放大后板上方内面 ≈ 286）
ADP_PRESS = dict(x=(73.0, 80.0), ys=(12.0, -12.0), w=2.0, gap=0.3)
# hr41（复审 #1 M1）：F06 压筋改叉形 —— 每条压筋在 −x 侧加一根齿夹住板顶的 −x 面（板 −x 面 72.5，齿 71.0..72.2 → 隙 0.3），
#   齿从压筋底往下 3.3（z 262.4..265.7），齿与压筋之间 x 72.2..73.0 在 z ≥ 265.7 补一段连上（hr39d probe_misc.json：齿盒对件、线都 0）。
#   原来只挡 z：板顶边 x 向没限位，h 6 的槽壁下可倾 6–10°，板顶偏 3–5 mm 压 1.9 缝里的 FPC。
ADP_TINE = dict(x=(71.0, 72.2), z0=262.4, bridge_x=(71.0, 73.2))
CEIL_PITCH = 0.5                              # 内面采样间距


_CEIL = {}
def _ceiling_solid(x0, x1, y0, y1, z_lo):
    """『顶壳内面以下』实体：顶面 = 原版 top_head_shell 内表面 + EMBED（射线逐点量，CEIL_PITCH 网格，线性插值），底面 z_lo。
    从内面长料的每一块都 inter 它一次 → 顶端埋进壁里 EMBED、绝不穿出外表面。"""
    key = (x0, x1, y0, y1, z_lo)
    if key in _CEIL: return _CEIL[key].copy()
    top = HS.scaled_orig("top_head_shell")                     # hr39c：放大后的顶壳（原来是原版 top_head_shell）
    xs = np.arange(x0, x1 + 1e-9, CEIL_PITCH); ys = np.arange(y0, y1 + 1e-9, CEIL_PITCH)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    inner = np.full(X.size, np.nan)
    todo = np.arange(X.size)
    for jx, jy in ((0.0137, 0.0071), (-0.0113, 0.0163), (0.0191, -0.0127)):   # 原版网格在 y=0 有一条棱，射线正落在棱上会只命中一次 → 微移后重投
        if not len(todo): break
        o = np.stack([X.ravel()[todo] + jx, Y.ravel()[todo] + jy, np.full(len(todo), 320.0)], 1); d = np.tile([0.0, 0.0, -1.0], (len(o), 1))
        loc, ridx, _ = top.ray.intersects_location(o, d, multiple_hits=True)
        rest = []
        for k, i in enumerate(todo):
            zs = np.sort(loc[ridx == k][:, 2])[::-1]
            zs = zs[np.concatenate([[True], np.diff(zs) < -0.05])] if len(zs) else zs   # 去掉落在棱上的重复命中
            if len(zs) < 2: rest.append(i); continue
            wall = zs[0] - zs[1]
            assert 1.4 <= wall <= 4.2, f"ceiling ({o[k][0]:.1f},{o[k][1]:.1f}) 壁厚 {wall:.2f} 不像顶壳顶板"   # 原版内面有两条横筋（x 52.2..53.1 / 12.7..13.6，1 宽 2 深）：筋处 3.7
            inner[i] = zs[1]
        todo = np.array(rest, dtype=int)
    assert not len(todo), f"ceiling 有 {len(todo)} 个采样点射线命中不足 2 次，如 ({X.ravel()[todo[0]]:.1f},{Y.ravel()[todo[0]]:.1f})"
    ztop = (inner + EMBED).reshape(X.shape)
    nx, ny = X.shape; off = X.size
    V = np.vstack([np.stack([X.ravel(), Y.ravel(), ztop.ravel()], 1), np.stack([X.ravel(), Y.ravel(), np.full(X.size, float(z_lo))], 1)])
    idx = lambda i, j: i * ny + j
    F = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            a, b, c, dd = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
            F += [[a, b, c], [a, c, dd], [a + off, c + off, b + off], [a + off, dd + off, c + off]]
    for i in range(nx - 1):
        a, b = idx(i, 0), idx(i + 1, 0);           F += [[a, a + off, b + off], [a, b + off, b]]
        a, b = idx(i, ny - 1), idx(i + 1, ny - 1); F += [[a, b + off, a + off], [a, b, b + off]]
    for j in range(ny - 1):
        a, b = idx(0, j), idx(0, j + 1);           F += [[a, b + off, a + off], [a, b, b + off]]
        a, b = idx(nx - 1, j), idx(nx - 1, j + 1); F += [[a, a + off, b + off], [a, b + off, b]]
    m = trimesh.Trimesh(vertices=V, faces=np.array(F), process=False)
    trimesh.repair.fix_normals(m)
    assert m.is_volume, "ceiling solid 不是闭合实体"
    _CEIL[key] = m
    return m.copy()


def _hull(pts):
    return trimesh.convex.convex_hull(np.asarray(pts, float))


def _prow_bar(x0, x1, y0, y1, z0, z1, prow=True):
    """沿 x 的直条 x0..x1 × y0..y1 × z0..z1；prow=True 时 +x 端接 45° 船头（顶点 x1 + (y1−y0)/2，在 y 中线），+x 面就不再是悬垂面。"""
    pts = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    if prow:
        pts += [(x1 + (y1 - y0) / 2, (y0 + y1) / 2, z) for z in (z0, z1)]
    return _hull(pts)


def _tab(x0, x1, y_rail, inward, z0, z1, sign):
    """卡爪：贴在导轨内面 y_rail（sign = 导轨在 ±y 哪一侧），向内伸 inward；顶面平（z1，托住件的底面），下面是斜面（从爪尖 z1−0.4 斜到轨根 z0）当导入面。"""
    y_tip = y_rail - sign * inward
    pts = [(x, y_rail, z) for x in (x0, x1) for z in (z0, z1)] + [(x, y_tip, z) for x in (x0, x1) for z in (z1 - 0.4, z1)]
    return _hull(pts)


def speaker_mount_solids():
    """F03：喇叭卡座各块（未与顶壳 inter 前的毛坯，顶端都开到 z 285，由 _ceiling_solid 截）。"""
    cx, cy = SPK_C; r = E.SPK_D / 2 + SPK_CLR; t = E.SPK_T + SPK_CLR
    z_bot = SPK_Z_SEAT - t - SPK_TAB_T                       # 爪底（hr38 263.9；hr39c 276.15）
    z_tab_top = SPK_Z_SEAT - t                               # 爪顶面（hr38 264.9）
    Z = 300.0
    out = []
    for s in (-1, 1):
        y_in = cy + s * r                                    # 导轨内面 ±14.2
        y_out = y_in + s * SPK_RAIL_T[s]
        out.append(_prow_bar(SPK_RAIL_X[0], SPK_RAIL_X[1], min(y_in, y_out), max(y_in, y_out), z_bot, Z))
        y_fl = y_in - s * SPK_FLANGE_IN                      # 肩内缘 ±12.0
        # was_until_2026_09_25_hr43d: out.append(_prow_bar(SPK_FLANGE_X[0], SPK_FLANGE_X[1], min(y_in, y_fl), max(y_in, y_fl), SPK_Z_SEAT, Z))
        out.append(_prow_bar(SPK_FLANGE_X[0], SPK_FLANGE_X[1], min(y_in, y_fl), max(y_in, y_fl), SPK_Z_SEAT + SPK_FOAM_RAISE, Z))   # hr43d：肩面抬 0.5 留泡棉环位
        for (x0, x1, inw) in SPK_TABS[s]:
            out.append(_tab(x0, x1, y_in, inw, z_bot, z_tab_top, s))
    xr0, hw = SPK_STOP_REAR                                  # 后挡：船头顶点 = 喇叭后缘 x 34.8
    out.append(_hull([(xr0, cy + sy * hw, z) for sy in (-1, 1) for z in (z_bot, Z)] + [(cx - r, cy, z) for z in (z_bot, Z)]))
    xf0 = cx + r; ft, hf = SPK_STOP_FRONT                    # 前挡：接触面 x 64.2，体 1.0 厚，不船头（见常量注释）
    out.append(wbox((xf0, cy - hf, z_bot), (xf0 + ft, cy + hf, Z)))
    return out


def amp_mount_solids_was_until_2026_09_25_hr43():
    """（hr43 起不用：功放搬到脸后口袋 H07 托板上）F04：功放板卡座各块（毛坯，顶端 z 300）。hr39c：AMP_TABS / AMP_FLANGE / 立筋 的 x 都是 hr38 板位下的值，整组 + E.T_AMP[0]。"""
    (ax0, ax1), (ay0, ay1), (az0, az1) = E.AMP_PCB
    dx = E.T_AMP[0]
    z_bot = az0 - AMP_CLR - AMP_TAB_T                        # 爪底（hr38 250.8）
    z_tab_top = az0 - AMP_CLR                                # 爪顶面（hr38 251.8）
    z_seat = az1 + AMP_CLR                                   # 肩面（hr38 253.8）
    Z = 300.0
    out = []
    for s in (-1, 1):
        y_in = (ay1 if s > 0 else ay0) + s * AMP_CLR         # 导轨内面 ±9.2
        y_out = y_in + s * AMP_RAIL_T[s]
        out.append(_prow_bar(ax0, ax1 + 0.7, min(y_in, y_out), max(y_in, y_out), z_bot, Z))
        y_fl = y_in - s * AMP_FLANGE_IN                      # 肩内缘 ±7.4
        for (fx0, fx1) in AMP_FLANGE[s]:
            out.append(_prow_bar(fx0 + dx, fx1 + dx, min(y_in, y_fl), max(y_in, y_fl), z_seat, Z))
        for (x0, x1, inw) in AMP_TABS[s]:
            out.append(_tab(x0 + dx, x1 + dx, y_in, inw, z_bot, z_tab_top, s))
        lt, (ly0, ly1) = AMP_LIP_FRONT                       # +x 唇：板边外 0.2 起，只到板顶以上（挡板的上半截即可）
        lo, hi = (ly0, ly1) if s > 0 else (-ly1, -ly0)
        out.append(_prow_bar(ax1 + AMP_CLR, ax1 + AMP_CLR + lt, lo, hi, az0 + 0.2, Z))
    (rx0, rx1), rhw, rfoot = AMP_STOP_REAR
    rx0, rx1 = rx0 + dx, rx1 + dx
    out.append(_prow_bar(rx0, rx1, -rhw, rhw, z_bot, Z))                                            # 立筋（3 宽 → 船头顶点 −10.7，离板边 0.7）
    out.append(_prow_bar(rx1 - 0.01, ax0 - AMP_CLR - rhw, -rhw, rhw, z_bot, z_bot + rfoot))   # 底脚：船头顶点落在 x = 板边 − 0.2
    return out


def vent_cutters():
    x0, x1 = VENT_X
    out = [wbox((x0, y - VENT_W / 2, VENT_Z[0]), (x1, y + VENT_W / 2, VENT_Z[1])) for y in VENT_YS]
    px0, px1 = POCKET_VENT_X                                   # hr39c F05
    out += [wbox((px0, y - VENT_W / 2, POCKET_VENT_Z[0]), (px1, y + VENT_W / 2, POCKET_VENT_Z[1])) for y in POCKET_VENT_YS]
    return out


def adapter_press_solids():
    """hr39c F06：转接板顶边压筋毛坯（顶端 z 300，由 _ceiling_solid 截）"""
    from .head import H03_ADP_Z
    a = ADP_PRESS; z0 = H03_ADP_Z[1] + a["gap"]
    return [_prow_bar(a["x"][0], a["x"][1], y - a["w"] / 2, y + a["w"] / 2, z0, 300.0) for y in a["ys"]]


def adapter_tine_solids():
    """hr41 F06 叉形 −x 齿（毛坯顶到 z 300，由 _ceiling_solid 截）。打印（世界 +x 朝下）：齿 +x 面在压筋底以下那 3.3 mm 是侧向悬垂，L1 切片待核。"""
    from .head import H03_ADP_Z
    a, t = ADP_PRESS, ADP_TINE; out = []
    for y in a["ys"]:
        y0, y1 = y - a["w"] / 2, y + a["w"] / 2
        out.append(wbox((t["x"][0], y0, t["z0"]), (t["x"][1], y1, 300.0)))                                   # 齿
        out.append(wbox((t["bridge_x"][0], y0, H03_ADP_Z[1] + a["gap"]), (t["bridge_x"][1], y1, 300.0)))   # 齿 ↔ 压筋连接（压筋底同高）
    return out


# ── hr41 H05 保持螺丝（fasteners F33_H05_keep）：H05 两侧各一个凸台，M2×8 自攻从 H03 地板底下 +z 拧进来（螺丝头在 H03 底面，被 J01 下巴盖住）。
#   为什么要：hr39c 里 H05 唯一的"锁"是 J01 毂卡在 H05 Ø21 圆让位里；A 方案 +y 必须开 21 槽，−y 的 J03 盘上方 H05 实测几乎没料
#   （盘上抬 3 mm 只交 0.11 mm³，hr41_work/scripts/probe_lock.py），H05 两侧都锁不住 → 两侧开槽 + 2 颗螺丝（任务给的备选），不做「只 +y 一颗」。
#   位置 x 64：J01 下巴在这里覆盖 |y| ≥ 33（头装好后看不见螺丝）；x 57 的嘴舵机端面外 4.5；Radxa/立柱/OTG 口/UBEC/麦克风都在 |y| ≤ 41 或 x ≥ 81.9。
#   |y| 48.5（初版 47.5 → 改）：−y 侧 12 V 主干（xt30_12v，hr41 M2 重走线）在 x 58..70 沿 y −44.5 过（宽 ±1.0），H05 竖直抽出时凸台从线下往上扫 → 47.5 时撞线 3.3 mm³
#   （hr41_work/seq_p_wires.json）；48.5 → 凸台内缘 |y| 46.0 离线 0.5。外侧：H03 合缝唇内面 |y| 51.33..51.55（到 z ≈235.7），凸台外缘 51.0 → 0.33；
#   J01 下缘内壁 |y| ≥ 51.0（张到 −5° 时壁顶抬到 ≈232.1）→ 头 Ø4（46.5..50.5）离 0.5。
#   凸台：Ø5 泪滴（+x 45° 尖，打印世界 +x 朝下不悬垂），底面 234.25 = H03 地板顶 233.95 + 0.3；z ≥ 236.3（唇顶 + 0.6）才长 45° 筋连到侧壁（埋进壁到 |y| 52.2）。
#   Ø1.7 底孔深 7.0：M2×8 − 叠厚（H03 地板 1.78 + 隙 0.3）= 咬入 5.9 ≥ pla_self_tap 4.0。
H05_KEEP_XY = ((64.0, 48.5), (64.0, -48.5))
H05_KEEP_XY_was_2026_09_24_hr41_draft = ((64.0, 47.5), (64.0, -47.5))
H05_KEEP = dict(d=5.0, z0=234.25, z1=243.0, web_z0=236.3, wall_y=52.2, web_x=(60.5, 72.3), pilot_d=1.7, pilot_depth=7.0)


def keep_boss_solids():
    k = H05_KEEP; r = k["d"] / 2; out = []
    for (x, y) in H05_KEEP_XY:
        s = 1.0 if y > 0 else -1.0
        ring = [(x + r * math.cos(t), y + r * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
        tip = (x + r * math.sqrt(2.0), y)
        low = [(px, py, z) for (px, py) in ring + [tip] for z in (k["z0"], k["web_z0"] + 0.01)]
        up = [(px, py, z) for (px, py) in ring + [tip, (k["web_x"][0], s * k["wall_y"]), (k["web_x"][1], s * k["wall_y"])] for z in (k["web_z0"], k["z1"])]
        out += [_hull(low), _hull(up)]
    return out


def keep_pilot_cutters():
    k = H05_KEEP
    return [cyl(k["pilot_d"], k["pilot_depth"] + 0.5, (x, y, k["z0"] - 0.5 + (k["pilot_depth"] + 0.5) / 2)) for (x, y) in H05_KEEP_XY]


def sound_hole_centers():
    p = SOUND_HOLE_PITCH; rows = p * math.sqrt(3) / 2
    cx, cy = SPK_C; out = []
    for j in range(-8, 9):
        y = cy + j * rows; xoff = p / 2 if j % 2 else 0.0
        for i in range(-8, 9):
            x = cx + i * p + xoff
            if math.hypot(x - cx, y - cy) <= SOUND_HOLE_RMAX + 1e-9: out.append((x, y))
    return out


def sound_hole_cutters():
    return [cyl(SOUND_HOLE_D, 14.0, (x, y, SPK_Z_SEAT + 5.0)) for (x, y) in sound_hole_centers()]   # hr39c：z 跟肩面走（hr38 中心 273 = 269.5 + 3.5）


def build_head_top_shell_was_until_2026_09_25_hr43():
    """（hr38..hr41 版，hr43 起不用）H05：原版 top_head_shell（hr39c：放大后）减通风槽（F01/F05）、减声孔（F02）、加喇叭卡座（F03）、加功放卡座（F04）、加转接板压筋（F06）；
    嘴：新枢轴孔 + 插入槽 + J01 −5..30° 扫掠让位（hr39c）。外表面除槽/孔/嘴让位外与放大后的原版逐面相同（合缝面、脸板凹槽都没动）。"""
    from . import jaw as JAW
    shell = HS.scaled_orig("top_head_shell")
    cx, cy = SPK_C
    ceil_spk = _ceiling_solid(round(cx - 19.0, 2), round(cx + 17.0, 2), round(cy - 20.0, 2), round(cy + 15.0, 2), round(SPK_Z_SEAT - 7.5, 2))
    (ax0, ax1), _, (az0, _) = E.AMP_PCB
    ceil_amp = _ceiling_solid(round(ax0 - 5.0, 2), round(ax1 + 3.0, 2), -12.5, 12.5, round(az0 - 3.0, 2))
    from .head import H03_ADP_Z
    ceil_adp = _ceiling_solid(70.0, 83.0, -15.0, 15.0, round(H03_ADP_Z[1] + 0.2, 2))
    ceil_tine = _ceiling_solid(70.5, 74.0, -15.0, 15.0, round(ADP_TINE["z0"] - 0.2, 2))                  # hr41 F06 叉形齿
    adds = [inter(b, ceil_spk) for b in speaker_mount_solids()] + [inter(b, ceil_amp) for b in amp_mount_solids_was_until_2026_09_25_hr43()] + [inter(b, ceil_adp) for b in adapter_press_solids()]
    adds += [inter(b, ceil_tine) for b in adapter_tine_solids()]
    adds += [inter(b, _shell_hull()) for b in keep_boss_solids()]                                         # hr41 保持螺丝凸台（F08）
    m = union(shell, *adds)
    m = diff(m, *vent_cutters(), *sound_hole_cutters(), wbox(*AMP_SHELL_WIN_was_until_2026_09_25_hr43), wbox(*AMP_TERM_WIN_was_until_2026_09_25_hr43))   # hr38：给功放弯排针的杜邦母壳在 −y 导轨上开窗；hr39c：+y 导轨给喇叭线开窗
    m = diff(m, *JAW.shell_pivot_cuts(slot=True), JAW.relief_sweep())            # hr39c 嘴
    m = diff(m, *JAW.h05_hub_slots(), *keep_pilot_cutters())                     # hr41：两侧 Ø21 圆让位向下开 21 槽到 z 225（F07 扩）；保持螺丝 Ø1.7 底孔
    return clean_print_topology(keep_main(m, "head_top_shell"))


_HULL = {}
def _shell_hull():
    """放大后顶壳顶点凸包（截保持螺丝凸台的筋，不让它冒出外表面；侧壁在 x 60..72 这一段外凸，凸包 ≈ 外表面）"""
    if "h" not in _HULL: _HULL["h"] = trimesh.convex.convex_hull(HS.scaled_orig("top_head_shell").vertices)
    return _HULL["h"].copy()


# ━━━━━━━━━━━━━━━━━━━━ hr43（2026-09-25）H05：Radxa 搬后脑顶部（方案 A）━━━━━━━━━━━━━━━━━━━━
#   用户 2026-09-24 17:25 定方案 A（docs/design_2026-09-17_bearing_rebuild/hr43_设计稿_Radxa后脑A.md）；17:20 定排针区上方按 ≥ 22 预留；18:20 定功放搬前口袋、喇叭座不动。
#   本轮 H05：F10 排针区鼓包（内面 ≥ 盒 A 顶 + 1.0）+ F11 四根吊柱（Radxa 从下拧）+ 长柱筋 + F12 SoC 上方 5 条出风槽；拆掉旧功放卡座 F04（函数/常量改名 *_was_until_2026_09_25_hr43）。
#   打印朝向不变（世界 +x 朝下、层沿 −x 堆，printability H05）：悬垂面 = 法向带 +x 分量 > cos45° 的面 →
#     ① 鼓包**内面**向 −x（后方）的坡、② 鼓包**外面**向 +x（前方）的坡，斜率都 ≤ 1；±y 两侧在打印里是竖壁，可以陡（把鼓包收窄）。
#     ③ 吊柱沿世界 z（打印时是横着的桩）→ +x 边做 45° 船头（泪滴，同 H05_KEEP）；筋沿 x、y 向只 1.6 宽（首层小悬垂，同 2 mm 槽封顶桥一级）。
#   鼓包做法（整体刚体不动，只让一块顶板「整片抬起」、壁厚按 2.0）：
#     盒 A 脚印 F0 = x −2..26 × y −28..24（electronics.ADP40_A_*）上内面要 ≥ z_in = 盒 A 顶 279.8 + 1.05 = 280.85 → 外面要 ≥ z_in + 壁 2.0 = 282.85（平台）；
#     （+1.05 而不是 +1.0：高度场 0.5 网格的斜坡起点三角面让盒 A 顶棱到前坡的欧氏距离少 0.02 —— build_fast #1 实测 0.982，改后 ≥1.0）
#     F0 外：平台往外降 G = G_x(dx) + G_y(dy)（dx/dy = 离 F0 的 x/y 距离）：G_x = 二次（曲率半径 rho_x）到斜率 1 后线性（①②）；G_y = dy²/(2 rho_y)（两侧陡）；
#     新外面 = smax(原外面, 平台 − G)（C1 光滑取大，k）；鼓包区（抬起量 δ > 0.02）里新内面 = 新外面 − 2.0（顶板整片抬起、原版两条纵向内筋 y ±9 与
#     横筋 x≈10 在鼓包区里一起削掉 —— 纵筋 x −16..5.5 挂到 z 262.3，正压在盒 A 与 SoC 上方）；鼓包区外一个点都不动。
#   量值（hr43_work/h05_bulge.json 实算）见报告。抬多少由 E.ADP40_TOP 推：客服给出「套上后离板面最高」后改那一个数，鼓包跟着变。
HR43_BULGE = dict(fx=(E.ADP40_A_OVER[0][0], E.ADP40_A_BEYOND[1][0]), fy=(E.ADP40_A_OVER[0][1], E.ADP40_A_OVER[1][1]),
                  z_in=E.ADP40_TOP + 1.05, wall=2.0, rho_x=2.0, dx_steep=16.0, steep=4.0, rho_y=1.5, k=1.0, eps=0.02,
                  grid_x=(-30.0, 36.0), grid_y=(-36.0, 36.0), pitch=0.5, add_embed=0.4, cut_depth=25.0)
#   dx_steep/steep：x 向坡在离 F0 16 mm 后改斜率 4 —— 后脑两角原壳本身就以 ≈1 的坡往下落（y −28 线上内面 −2→−22 从 272.9 落到 261.9），
#   斜率 1 的坡追不上它、会一路贴着壳伸到后脑勺；16 mm 时坡已经没进原壳里（鼓包区内内面 +x 向坡 max 1.0，hr43 调参），之后改陡只在原壳里面、看不见。
# F11 吊柱：板孔 E.SBC_A_HOLES（58×23）；Ø6 泪滴（+x 45° 船头）；柱端贴板顶 E.SBC_A_TOP 257.8（板从下贴柱端，M2 自攻 + 小平垫从板底拧进柱，F37）。
#   柱端 3.4 段（z 257.8..261.2）缩成 Ø5.4 圆柱、不带船头：官方 DXF 孔心 r 3.0 内正面无元件（E.SBC_HOLE_KEEPOUT_R）→ 离禁布圈 0.3；该段以上（261.2 起）才是 Ø6 泪滴，
#   离顶面元件包络顶 260.8 有 0.4、离 −x 边口壳顶 261.0 有 0.2。
#   Ø1.7 底孔深 7.0（M2×8 − 板 1.6 − 小平垫 0.3 = 咬入 6.1 ≥ pla_self_tap 4.0；口径同 H05_KEEP / H04_CLAMP_BOSS）。
#   (−14.5, −31) 柱端在 OTG 口壳旁削平（离口壳 0.2，同 hr39c H01_OTG_RELIEF 的口径：口壳按规范 8.94 全宽，离孔心 2.33 < DXF 3.0）；
#   (8.5, ±) 两根柱贴盒 A 的 ±y 面那侧削平到离盒 A 0.5（孔壁到平面 1.65 ≥ 1.2）。
#   长柱（>10）两侧沿 x 各一片筋（y 1.6 宽，下沿从柱身 262.5 起 45° 往外抬）连到顶板：−x 片长 7、外端竖直（打印时朝上）；
#   +x 片（打印时朝床那侧）不留竖直外端 —— 45° 下沿一直延到顶壳内面（L_plus 20 只是上限，实际由 +x 片自己那条 _ceiling_from 截在内面 ≈15 处）。
#   （hr43 打印性粗核 09-25：原 +x 片长 7 的竖直外端 3 片共 32.8 mm² 平悬垂 → 改 45° 下沿延到内面；hr43_work/overhang_clusters.json）
H05_POSTS_was_until_2026_09_25_hr43c = dict(xy=((-14.5, -31.0), (8.5, -31.0), (-14.5, 27.0), (8.5, 27.0)), d=6.0, d_tip=5.4, z0=257.8, z_body=261.2, pilot_d=1.7, pilot_depth=7.0,
                 otg_relief=dict(post=(-14.5, -31.0), box=((-19.4, -28.87, 256.0), (-11.4, -20.0, 261.2))),
                 adp_flat=0.5, fin=dict(t=1.6, L=7.0, L_plus=20.0, z_at_post=262.5, min_len=10.0))
# hr43c（2026-09-25）：板整体平移 E.HR43C_SBC_SHIFT (−0.5, +1.0, −0.5) → 柱心随板孔、柱端随板顶（257.3，柱加长 0.5）、柱端 Ø5.4 段 z 随板、OTG 口旁削平盒随板；
#   盒 A（zz_adapter40）作废 → 两根前柱 (8, ±) 贴的是直插杜邦壳（electronics.gpio_dupont_boxes，十字并集 y −26.53..24.53）：柱在壳一侧削平到离壳 hdr_flat 1.0
#   （孔壁到平面 = 2.47 − 0.85 = 1.62 ≥ 1.2；hr43 盒 A 口径 0.5 → 1.0，复审「吊柱安装界面 <1.0」那条少一处）。
_S = E.HR43C_SBC_SHIFT
H05_POSTS = dict(xy=E.SBC_A_HOLES, d=6.0, d_tip=5.4, z0=E.SBC_A_TOP, z_body=round(E.SBC_A_TOP + 3.4, 6), pilot_d=1.7, pilot_depth=7.0,
                 otg_relief=dict(post=(round(-14.5 + _S[0], 6), round(-31.0 + _S[1], 6)),
                                 box=tuple(tuple(round(v + d, 6) for v, d in zip(c, _S)) for c in ((-19.4, -28.87, 256.0), (-11.4, -20.0, 261.2)))),
                 hdr_flat=1.0, fin=dict(t=1.6, L=7.0, L_plus=20.0, z_at_post=262.5, min_len=10.0))
# F12 SoC 出风槽：SoC 包络（design_A_optimize 的 heatsink_assumed 经变换 = x −12.6..2.4 × y −5.6..9.4，顶 258.5..259.8）正上方；
#   照 vent_cutters 的写法（沿 x 长、y 向窄 → 打印每层只是缺口，端头是 1.6 mm 小桥）；5 条 1.6 宽、筋 1.4，y −6..6 避开保留的原版纵筋（已在鼓包区削掉，这里只是不贴）；
#   x −13..−3 止于盒 A（x ≥ −2）后 1 mm。槽穿过的是抬起后的顶板（鼓包后坡），不穿柱/筋。
# F14 Radxa 板角让位（hr43 09-25 复核补）：后脑 −y 后角原壳内面离板角只有 0.734（manifold，build_fast #3；设计稿 §2「静态最近 顶壳 0.63（板角）」是同一处），
#   任务书验证 3 要 Radxa ↔ 顶壳 ≥ 1.0 → 板外廓（x −18..12 × y −34.5..30.5 × z 256.2..257.8，四角 R3 同 electronics.sbc_boxes）的**真等距外扩 grow**
#   （四角圆弧点 × 上下两面、每点一个 r = grow 的球、取凸包 = 圆角板的精确 offset）从壳上挖掉，四柱 r post_keep_r 内不挖（柱端贴板）。
#   只在后 −y 角动到料（其余处壳离板本来 > grow），挖深 ≤ grow − 0.734；剩壁见 hr43_work/relief_check.json。
#   （先试过「板外廓盒各向 +1.05」：盒角比真 offset 多挖到 √3 倍，后角斜壁剩 0.255 → 弃用）
HR43_SBC_RELIEF = dict(grow=1.05, post_keep_r=3.2, board_corner_r=3.0, arc_pts=13)
HR43_SOC_VENT = dict(x=(-13.0, -3.0), ys=(-6.0, -3.0, 0.0, 3.0, 6.0), w=1.6, z=(262.0, 300.0))

_BG = {}
def _roof_grid():
    """放大后原版顶壳在 HR43_BULGE 网格上的外面 OUT / 第二次命中 IN（内面或原版内筋底），射线从 z 320 往下（同 _ceiling_solid 的防棱重投）。向量化分组。"""
    if "g" in _BG: return _BG["g"]
    b = HR43_BULGE; top = HS.scaled_orig("top_head_shell")
    xs = np.arange(b["grid_x"][0], b["grid_x"][1] + 1e-9, b["pitch"]); ys = np.arange(b["grid_y"][0], b["grid_y"][1] + 1e-9, b["pitch"])
    X, Y = np.meshgrid(xs, ys, indexing="ij"); n = X.size
    OUT = np.full(n, np.nan); IN = np.full(n, np.nan); todo = np.arange(n)
    for jx, jy in ((0.0137, 0.0071), (-0.0113, 0.0163), (0.0191, -0.0127)):
        if not len(todo): break
        rest = []
        for s in range(0, len(todo), 4000):
            idx = todo[s:s + 4000]
            o = np.stack([X.ravel()[idx] + jx, Y.ravel()[idx] + jy, np.full(len(idx), 320.0)], 1); d = np.tile([0.0, 0.0, -1.0], (len(idx), 1))
            loc, ridx, _ = top.ray.intersects_location(o, d, multiple_hits=True)
            order = np.lexsort((-loc[:, 2], ridx)); loc, ridx = loc[order], ridx[order]
            starts = np.searchsorted(ridx, np.arange(len(idx))); ends = np.searchsorted(ridx, np.arange(len(idx)), side="right")
            for k in range(len(idx)):
                zs = loc[starts[k]:ends[k], 2]
                if len(zs): zs = zs[np.concatenate([[True], np.diff(zs) < -0.05])]
                if len(zs) < 2: rest.append(idx[k]); continue
                OUT[idx[k]], IN[idx[k]] = zs[0], zs[1]
        todo = np.array(rest, dtype=int)
    assert not len(todo), f"HR43_BULGE 网格有 {len(todo)} 个点射线命中不足 2 次（网格出了壳的投影），如 ({X.ravel()[todo[0]]:.1f},{Y.ravel()[todo[0]]:.1f})"
    _BG["g"] = (xs, ys, OUT.reshape(X.shape), IN.reshape(X.shape))
    return _BG["g"]


def _smax(a, b, k):
    h = np.clip(k - np.abs(a - b), 0.0, None)
    return np.maximum(a, b) + h * h / (4.0 * k)


def bulge_fields():
    """(xs, ys, OUT, IN, delta, new_out)：delta = 外面抬起量（≥0），new_out = 原外面 + delta。"""
    if "f" in _BG: return _BG["f"]
    b = HR43_BULGE; xs, ys, OUT, IN = _roof_grid()
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    (fx0, fx1), (fy0, fy1) = b["fx"], b["fy"]
    dx = np.maximum(np.clip(fx0 - X, 0, None), np.clip(X - fx1, 0, None))
    dy = np.maximum(np.clip(fy0 - Y, 0, None), np.clip(Y - fy1, 0, None))
    rx, ds = b["rho_x"], b["dx_steep"]
    Gx = np.where(dx <= rx, dx * dx / (2 * rx), np.where(dx <= ds, rx / 2 + (dx - rx), rx / 2 + (ds - rx) + b["steep"] * (dx - ds)))   # 二次 → 斜率 1（①②）→ 16 后陡
    Gy = dy * dy / (2 * b["rho_y"])
    pillow = b["z_in"] + b["wall"] - Gx - Gy
    new_out = _smax(OUT, pillow, b["k"])
    delta = new_out - OUT
    # 网格边上必须已经回到原壳（鼓包整个落在网格里，堆出来的料不会在网格边上断开）
    edge = np.concatenate([delta[0], delta[-1], delta[:, 0], delta[:, -1]])
    assert edge.max() < b["eps"], f"鼓包伸出 HR43_BULGE 网格边（边上 δ max {edge.max():.3f}）"
    _BG["f"] = (xs, ys, OUT, IN, delta, new_out)
    return _BG["f"]


def _heightfield(xs, ys, ztop, zbot):
    """规则网格高度场闭合实体（顶面 ztop、底面 zbot，二维数组 [ix, iy]；要求 ztop > zbot 处处）。同 _ceiling_solid 的拼法。"""
    X, Y = np.meshgrid(xs, ys, indexing="ij"); nx, ny = X.shape; off = X.size
    assert np.all(ztop - zbot > 1e-4), "heightfield 顶底相交"
    V = np.vstack([np.stack([X.ravel(), Y.ravel(), ztop.ravel()], 1), np.stack([X.ravel(), Y.ravel(), zbot.ravel()], 1)])
    I = np.arange(off).reshape(nx, ny)
    a, bq, c, d = I[:-1, :-1].ravel(), I[1:, :-1].ravel(), I[1:, 1:].ravel(), I[:-1, 1:].ravel()
    F = [np.stack([a, bq, c], 1), np.stack([a, c, d], 1), np.stack([a + off, c + off, bq + off], 1), np.stack([a + off, d + off, c + off], 1)]
    for e0, e1 in ((I[:-1, 0], I[1:, 0]), (I[1:, -1], I[:-1, -1]), (I[-1, :-1], I[-1, 1:]), (I[0, 1:], I[0, :-1])):   # 四条边（逆时针一圈）
        F += [np.stack([e0, e0 + off, e1 + off], 1), np.stack([e0, e1 + off, e1], 1)]
    m = trimesh.Trimesh(vertices=V, faces=np.vstack(F), process=False)
    trimesh.repair.fix_normals(m)
    assert m.is_volume, "heightfield 不是闭合实体"
    return m


def bulge_add_cut():
    """F10 鼓包的 (加料, 减料) 两块实体（世界系）。加料 = 原外面 −0.4 .. 新外面；减料 = 新内面往下 cut_depth（削掉原顶板下半截与鼓包区里的原版内筋）。
    δ ≤ eps 的格子：加料退成原外面下 0.05..0.45 的薄片（在壁里，union 不变）、减料退成原内面下 0.05..1.05（空气里，diff 不变）。"""
    b = HR43_BULGE; xs, ys, OUT, IN, delta, new_out = bulge_fields()
    on = delta > b["eps"]
    add_top = np.where(on, new_out, OUT - 0.05); add_bot = np.where(on, OUT - b["add_embed"], OUT - 0.45)
    cut_top = np.where(on, new_out - b["wall"], IN - 0.05); cut_bot = np.where(on, cut_top - b["cut_depth"], IN - 1.05)
    return _heightfield(xs, ys, add_top, add_bot), _heightfield(xs, ys, cut_top, cut_bot)


_BS = {}
def hr43_bulged_shell():
    """放大后的原版顶壳 + F10 鼓包（后面所有加料/减料都在它上面做）。同进程缓存。"""
    if "s" not in _BS:
        add, cut = bulge_add_cut()
        _BS["s"] = diff(union(HS.scaled_orig("top_head_shell"), add), cut)
    return _BS["s"].copy()


def _ceiling_from(mesh, x0, x1, y0, y1, z_lo, embed=EMBED, wall=(1.4, 6.0)):
    """同 _ceiling_solid，但对给定网格（鼓包后的顶壳）量内面：顶面 = 内面 + embed。wall = 竖直射线穿壁厚的合理范围（陡侧壁斜穿会到 3–5，原版内筋处会更厚 → 报错）。"""
    xs = np.arange(x0, x1 + 1e-9, CEIL_PITCH); ys = np.arange(y0, y1 + 1e-9, CEIL_PITCH)
    X, Y = np.meshgrid(xs, ys, indexing="ij"); inner = np.full(X.size, np.nan); todo = np.arange(X.size)
    for jx, jy in ((0.0137, 0.0071), (-0.0113, 0.0163), (0.0191, -0.0127)):
        if not len(todo): break
        o = np.stack([X.ravel()[todo] + jx, Y.ravel()[todo] + jy, np.full(len(todo), 320.0)], 1); d = np.tile([0.0, 0.0, -1.0], (len(o), 1))
        loc, ridx, _ = mesh.ray.intersects_location(o, d, multiple_hits=True)
        order = np.lexsort((-loc[:, 2], ridx)); loc, ridx = loc[order], ridx[order]
        st = np.searchsorted(ridx, np.arange(len(todo))); en = np.searchsorted(ridx, np.arange(len(todo)), side="right"); rest = []
        for k, i in enumerate(todo):
            zs = loc[st[k]:en[k], 2]
            if len(zs): zs = zs[np.concatenate([[True], np.diff(zs) < -0.05])]
            if len(zs) < 2: rest.append(i); continue
            assert wall[0] <= zs[0] - zs[1] <= wall[1], f"ceiling ({X.ravel()[i]:.1f},{Y.ravel()[i]:.1f}) 壁厚 {zs[0] - zs[1]:.2f} 不像顶板"
            inner[i] = zs[1]
        todo = np.array(rest, dtype=int)
    assert not len(todo), f"ceiling 有 {len(todo)} 个点命中不足 2 次"
    ztop = np.maximum((inner + embed).reshape(X.shape), float(z_lo) + 0.05)     # 内面低于 z_lo 的角落（陡侧壁）压成薄片：那里本来就不长料
    return _heightfield(xs, ys, ztop, np.full(X.shape, float(z_lo)))


def _teardrop_pts(x, y, r):
    """Ø2r 圆 + +x 45° 船头（尖在 x + r√2）的 2D 轮廓点"""
    ring = [(x + r * math.cos(t), y + r * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
    return ring + [(x + r * math.sqrt(2.0), y)]


# hr52（2026-09-30）：两根前吊柱的削平面原来跟 electronics.gpio_dupont_boxes() 的包围盒走；hr52 线束轮 1 号针（ToF VIN）腿改朝 +x 偏 +y 37°，
#   包围盒 y 上沿 24.53 → 24.893，重建会把 +y 柱削平面从 25.53 推到 25.893（H05 跟着变、要重打顶壳）。H05 是 v4 已发店家的件，不为线净空重打 →
#   钉在 hr52 之前的包围盒（逐位 = 旧 electronics 算出的 bounds，hr52_work 探针算得）；代价：VIN 线离 +y 柱削平面 0.096（软线，线束轮已核不相交）。
H05_HDR_FLAT_BOUNDS = ((4.884217739105225, -26.530000686645508, 259.79998779296875), (13.522322654724121, 24.530000686645508, 278.5982971191406))


def hr43_post_solids():
    """F11 四根吊柱 + 长柱筋的毛坯（顶端开到 z 300，由鼓包后顶壳的 _ceiling_from 截）。返回 [(柱或筋, 截用的 xy 盒)]"""
    p = H05_POSTS; out = []
    # was_until_2026_09_25_hr43c: (ax0, ay0, _), (ax1, ay1, _) = E.ADP40_A_OVER
    # was_until_2026_09_30_hr52: (ax0, ay0, _), (ax1, ay1, _) = E.gpio_dupont_boxes().bounds                # hr43c：直插杜邦壳 + 弯线管占位的包围盒（x 5.53..11.52、y −26.53..24.53）
    (ax0, ay0, _), (ax1, ay1, _) = H05_HDR_FLAT_BOUNDS                          # hr52：钉在 hr52 之前的包围盒（见 H05_HDR_FLAT_BOUNDS 注），H05 不因排针改线而变
    for (x, y) in p["xy"]:
        tip = cyl(p["d_tip"], p["z_body"] - p["z0"], (x, y, (p["z0"] + p["z_body"]) / 2), sections=48)
        body = _hull([(px, py, z) for (px, py) in _teardrop_pts(x, y, p["d"] / 2) for z in (p["z_body"], 300.0)])
        post = union(tip, body)
        if (x, y) == tuple(p["otg_relief"]["post"]):
            post = diff(post, wbox(*p["otg_relief"]["box"]))
        # was_until_2026_09_25_hr43c: if ax0 - 3.0 <= x <= E.ADP40_A_BEYOND[1][0] + 3.0:   # 贴盒 A ±y 面那一侧削平（离盒 A 0.5）
        # was_until_2026_09_25_hr43c:     post = diff(post, wbox((x - 6, ay0 - p["adp_flat"], 250.0), (x + 6, ay1 + p["adp_flat"], 301.0)))
        if ax0 - 3.0 <= x <= ax1 + 3.0:                                         # hr43c：贴杜邦壳那一侧削平（离壳 1.0）
            post = diff(post, wbox((x - 6, ay0 - p["hdr_flat"], 250.0), (x + 6, ay1 + p["hdr_flat"], 301.0)))
        out.append(post)
    return out


def hr43_fin_solids(post_len, only=None, side=None):
    """长柱筋：每根柱长 > fin.min_len 时，沿 ±x 各一片（y 宽 t）；下沿在柱身处 z_at_post，往外 45° 抬（顶端由 _ceiling_from 截）。post_len = {(x,y): 长}
    −x 片长 L（外端竖直，打印时朝上）；+x 片长 L_plus（上限；45° 下沿延到内面为止，不留朝床的竖直外端）。side = −1 / +1 只要一侧，None 两侧。"""
    p = H05_POSTS; f = p["fin"]; out = []
    for (x, y) in p["xy"]:
        if only is not None and (x, y) != tuple(only): continue
        if post_len[(x, y)] <= f["min_len"]: continue
        for s in (-1, 1):
            if side is not None and s != side: continue
            xa, xb = x, x + s * (f["L"] if s < 0 else f["L_plus"])
            lo, hi = min(xa, xb), max(xa, xb)
            pts = [(xx, yy, z) for yy in (y - f["t"] / 2, y + f["t"] / 2) for (xx, z) in ((xa, f["z_at_post"]), (xa, 300.0), (xb, 300.0), (xb, f["z_at_post"] + abs(xb - xa)))]
            out.append(_hull(pts))
    return out


def hr43_pilot_cutters():
    p = H05_POSTS
    return [cyl(p["pilot_d"], p["pilot_depth"] + 0.5, (x, y, p["z0"] - 0.5 + (p["pilot_depth"] + 0.5) / 2)) for (x, y) in p["xy"]]


def hr43_sbc_relief_cutters():
    """F14：Radxa 圆角板（R3）的真等距外扩 grow（球 × 圆弧点的凸包），扣掉四柱 r post_keep_r"""
    R = HR43_SBC_RELIEF; g, rc = R["grow"], R["board_corner_r"]; cx, cy, cz = E.SBC_A_C
    xa, xb = cx - E.SBC_W / 2 + rc, cx + E.SBC_W / 2 - rc; ya, yb = cy - E.SBC_L / 2 + rc, cy + E.SBC_L / 2 - rc
    ctr = []
    for (px, py, a0) in ((xb, yb, 0.0), (xa, yb, 90.0), (xa, ya, 180.0), (xb, ya, 270.0)):
        for a in np.radians(np.linspace(a0, a0 + 90.0, R["arc_pts"])):
            ctr += [(px + rc * math.cos(a), py + rc * math.sin(a), z) for z in (cz - E.SBC_T / 2, cz + E.SBC_T / 2)]
    ball = trimesh.creation.icosphere(subdivisions=3, radius=g).vertices
    off = trimesh.convex.convex_hull((np.asarray(ctr)[:, None, :] + ball[None, :, :]).reshape(-1, 3))
    keep = [cyl(2 * R["post_keep_r"], 40.0, (x, y, E.SBC_A_TOP + 10.0)) for (x, y) in E.SBC_A_HOLES]
    return [diff(off, *keep)]

# ━━ hr43c（2026-09-25）H05 后脑零鼓包：F10 鼓包退役（HR43_BULGE / bulge_* / hr43_bulged_shell 只留痕，不再加料）→ 外形 = 放大原版顶壳 ━━
#   用户 13:0x 定直插四件（hr43_work/agent_brief_hr43c.md；数据 无鼓包研究.md §2.5 / hr43c/c1_dy_sweep.json），本件做其中两件：
#   F10′（新）① 横筋局部削：原版顶壳 x 9.75..10.75 横筋（1 宽，筋底比内皮低 1.98..2.90）在排针上方 rib.x × rib.y 里削到内皮
#        —— 筋处按同 y 左右（interp_x 内非筋格）内皮线性插值，只动内面、外观不变；筋 = 竖直壁厚 > wall_min 的格（同 a4c_combo）。
#     ② 两端内皮局部减薄：±y 两端排针（针 1..6 / 35..40 的 y 带 ±1.6）× x（后排针 −1.6 .. 前排针 + 1.4 + Rc 2.8 + 3）里内皮往上抬 ≤ t 0.55，
#        且竖直壁厚 ≥ wall 1.2 × √(1 + |∇外面|²)（法向 1.2，assumed 最小可打印壁）；外面不动。
#   做法：0.25 网格竖直射线量原版外面 OUT / 内面 IN（同 _roof_grid 防棱微移重投）→ 刀的顶面 = 新内面（筋格 SKIN − eps、减薄格 SKIN + thin、其余 IN − 0.05 在空气里不切），
#     底面 = 顶面 − depth；刀在装吊柱之前从原版壳上减（吊柱 _ceiling_from 量的是削后内面）。
def _hr43c_pins_x():
    return E.gpio_pin_xy(1)[0], E.gpio_pin_xy(2)[0]          # 后排（奇数）x、前排（偶数）x


def _hr43c_thin_blocks():
    t = HR43C_ROOF["thin"]; xr, xf = _hr43c_pins_x(); y = lambda n: E.gpio_pin_xy(n)[1]
    x0, x1 = xr - t["pad"], xf + 1.4 + t["Rc_cover"] + t["run"]
    return ((x0, x1, y(6) - t["pad"], y(1) + t["pad"]), (x0, x1, y(40) - t["pad"], y(35) + t["pad"]))


HR43C_ROOF = dict(grid_x=(3.0, 19.0), grid_y=(-30.0, 27.0), pitch=0.25, depth=6.0,
                  rib=dict(x=(9.5, 11.0), y=(-29.0, 26.0), wall_min=2.4, interp_x=(7.0, 14.0), eps=0.02),
                  thin=dict(t=0.55, wall=1.2, pad=1.6, Rc_cover=2.8, run=3.0))
_RC = {}
def hr43c_roof_fields():
    """(xs, ys, OUT, IN, SKIN, THIN, rib_mask)：放大原版顶壳在 HR43C_ROOF 网格上的量（缓存）"""
    if "f" in _RC: return _RC["f"]
    R = HR43C_ROOF; top = HS.scaled_orig("top_head_shell")
    xs = np.arange(R["grid_x"][0], R["grid_x"][1] + 1e-9, R["pitch"]); ys = np.arange(R["grid_y"][0], R["grid_y"][1] + 1e-9, R["pitch"])
    X, Y = np.meshgrid(xs, ys, indexing="ij"); n = X.size
    OUT = np.full(n, np.nan); IN = np.full(n, np.nan); todo = np.arange(n)
    for jx, jy in ((0.0137, 0.0071), (-0.0113, 0.0163), (0.0191, -0.0127)):
        if not len(todo): break
        rest = []
        for s0 in range(0, len(todo), 4000):
            idx = todo[s0:s0 + 4000]
            o = np.stack([X.ravel()[idx] + jx, Y.ravel()[idx] + jy, np.full(len(idx), 320.0)], 1); d = np.tile([0.0, 0.0, -1.0], (len(idx), 1))
            loc, ridx, _ = top.ray.intersects_location(o, d, multiple_hits=True)
            order = np.lexsort((-loc[:, 2], ridx)); loc, ridx = loc[order], ridx[order]
            st = np.searchsorted(ridx, np.arange(len(idx))); en = np.searchsorted(ridx, np.arange(len(idx)), side="right")
            for k in range(len(idx)):
                zs = loc[st[k]:en[k], 2]
                if len(zs): zs = zs[np.concatenate([[True], np.diff(zs) < -0.05])]
                if len(zs) < 2: rest.append(idx[k]); continue
                OUT[idx[k]], IN[idx[k]] = zs[0], zs[1]
        todo = np.array(rest, dtype=int)
    assert not len(todo), f"HR43C_ROOF 网格有 {len(todo)} 个点射线命中不足 2 次"
    OUT = OUT.reshape(X.shape); IN = IN.reshape(X.shape); WALL = OUT - IN
    rb = R["rib"]; ribcol = (xs >= rb["x"][0]) & (xs <= rb["x"][1]); okcol = (xs >= rb["interp_x"][0]) & (xs <= rb["interp_x"][1])
    SKIN = IN.copy(); ribm = np.zeros_like(IN, bool)
    for j in range(len(ys)):
        if not (rb["y"][0] <= ys[j] <= rb["y"][1]): continue
        r = ribcol & (WALL[:, j] > rb["wall_min"]); ok = ~r & okcol
        if r.any():
            SKIN[r, j] = np.interp(xs[r], xs[ok], IN[ok, j]); ribm[r, j] = True
    gx, gy = np.gradient(OUT, xs, ys)
    tmax = np.clip(OUT - SKIN - R["thin"]["wall"] * np.sqrt(1.0 + gx ** 2 + gy ** 2), 0.0, R["thin"]["t"])
    inb = np.zeros_like(IN, bool)
    for (a, b, c, d) in _hr43c_thin_blocks(): inb |= (X >= a) & (X <= b) & (Y >= c) & (Y <= d)
    THIN = np.where(inb, tmax, 0.0)
    _RC["f"] = (xs, ys, OUT, IN, SKIN, THIN, ribm)
    return _RC["f"]


def hr43c_roof_cutter():
    """F10′ 刀（横筋局部削 + 两端内皮减薄）：高度场实体，顶面 = 新内面"""
    R = HR43C_ROOF; xs, ys, OUT, IN, SKIN, THIN, ribm = hr43c_roof_fields()
    top = np.where(ribm, SKIN - R["rib"]["eps"], IN - 0.05)
    top = np.where(THIN > 1e-3, SKIN + THIN, top)
    return _heightfield(xs, ys, top, top - R["depth"])


def hr43c_shell():
    """放大原版顶壳 − F10′（缓存）"""
    if "s" not in _RC: _RC["s"] = diff(HS.scaled_orig("top_head_shell"), hr43c_roof_cutter())
    return _RC["s"].copy()



def hr43_soc_vent_cutters():
    v = HR43_SOC_VENT
    return [wbox((v["x"][0], y - v["w"] / 2, v["z"][0]), (v["x"][1], y + v["w"] / 2, v["z"][1])) for y in v["ys"]]


def hr43_post_lengths(shell=None):
    """每根柱从板顶到鼓包后内面的长（射线实测，柱心）"""
    shell = shell if shell is not None else hr43_bulged_shell(); out = {}
    for (x, y) in H05_POSTS["xy"]:
        loc, _, _ = shell.ray.intersects_location([[x + 0.0137, y + 0.0071, H05_POSTS["z0"] + 0.01]], [[0, 0, 1.0]], multiple_hits=True)
        zs = sorted(l[2] for l in loc); out[(x, y)] = float(zs[0] - H05_POSTS["z0"])
    return out


# hr50（2026-09-28 用户：比一条挤出线还薄的薄片不要；默认收刀留 0.8，会碰运动件 / 别的件才整片削穿、切口平整）：
#   ① 两侧毂盘 Ø21 让位（J02 / J03 盘面外 0.5 的轴向间隙，jaw.shell_pivot_cuts）在壳壁内面离得近的一角（x 38.5..42.3、z 243..249，内面 y ±48.6..49.0）
#      只剩 0.05..0.45 的底皮（薄膜 20.6 / 4.8 mm²、尖刺 (41.3, −49.0, 244.6)）。收刀要吃进盘面间隙 → 会碰 → 这一角整片穿透：
#      Ø21 圆里 z ≤ 249.5（底皮厚 ≥0.8 的等高线：z 249 处 0.79..0.95）的壳壁切掉，切口 = Ø21 圆柱面 + z 249.5 平面，都垂直于壁；圆外壳壁不动。
#      穿透的这一角在盘（Ø20）背后，只从 0.5 的环缝里看得到。
H05_HUB_THRU_Z1 = 249.5
#   ② 合缝唇：J01 扫掠刀（2.5° 一步）从下面斜着削上壳侧壁下沿那条唇（y ±51.25..52.4、自然唇底 z 233.93）：x 48 → 43.8 唇底被削高到 234.0 → 235.75，
#      x 43.8..45.7 只剩 0.05（薄膜 2.8 / 2.7 mm²、尖刺 (44.6, −51.3, 234.1)），削面是扫掠锯齿（碎边 ×2）。补厚会进扫掠包络 →
#      被扫掠碰到的这一段唇（x 43.4..48.5）整段切平到 z 235.95（唇根，上面壁厚 1.25）；x 48.5 以外扫掠够不到唇，唇原样。
H05_SEAM_LIP_CUTS = (((43.4, 50.9, 233.5), (48.5, 52.6, 235.95)), ((43.4, -52.6, 233.5), (48.5, -50.9, 235.95)))
def h05_hr50_cuts():
    from . import jaw as JAW
    x0, z0 = JAW.JAW_AXIS_P[0], JAW.JAW_AXIS_P[2]
    hub = [inter(cyl(JAW.SHELL_HUB_RELIEF_D, 6.5, (x0, s * 47.75, z0), axis="y"),
                 wbox((x0 - 11.0, min(s * 44.4, s * 51.1), 225.0), (x0 + 11.0, max(s * 44.4, s * 51.1), H05_HUB_THRU_Z1))) for s in (1, -1)]
    return hub + [wbox(*b) for b in H05_SEAM_LIP_CUTS]


def build_head_top_shell():
    """H05（hr43）：放大后原版顶壳 + F10 鼓包 → 减通风槽（F01/F05）/ 声孔（F02）/ SoC 出风槽（F12）；加喇叭卡座（F03）、转接板压筋 + 叉形齿（F06/F09；hr43d 删）、
    保持螺丝凸台（F08）、Radxa 吊柱 + 筋（F11）；**不再有功放卡座 F04**（功放搬前口袋 H07 托板，旧版 = build_head_top_shell_was_until_2026_09_25_hr43）。
    嘴：新枢轴孔 + 插入槽 + J01 −5..30° 扫掠让位（hr39c）；两侧 21 槽 + 保持螺丝底孔（hr41）。鼓包外的外表面与放大后的原版逐面相同。"""
    from . import jaw as JAW
    # was_until_2026_09_25_hr43c: shell = hr43_bulged_shell()
    shell = hr43c_shell()                                                          # hr43c：零鼓包 = 放大原版顶壳 − F10′（横筋局部削 + 两端内皮减薄）
    cx, cy = SPK_C
    # was_until_2026_09_25_hr43b: ceil_spk = _ceiling_solid(round(cx - 19.0, 2), round(cx + 17.0, 2), round(cy - 20.0, 2), round(cy + 15.0, 2), round(SPK_Z_SEAT - 7.5, 2))
    # hr43b（G2，Lane D hr43_登记修正_L1L2.md §4）：y 上界写死 cy+15 = 18.06，把 +y 卡扣导轨（内面 cy+r = 17.26，SPK_RAIL_T[+1] 1.2 → 外面 18.46）截成 0.8 厚
    #   （扣入应力 / 刚度按 1.2 算的，见 SPK_RAIL_T 上面的注释）。改：上界由导轨外面推出 = cy + SPK_D/2 + SPK_CLR + SPK_RAIL_T[+1] + 2·CEIL_PITCH = 19.46
    #   （0.5 网格末列 19.06，导轨外面外 0.6）。ceil_spk 只拿来截 speaker_mount_solids：预演 9 块里只有 +y 导轨变（151.87 → 226.43 mm³，y 17.26..18.06 → 17.26..18.46），
    #   其余 8 块体积不变；壳 / 鼓包 / 四柱 / 槽孔都不经过 ceil_spk。_ceiling_solid 的壁厚断言在新范围内全过。
    ceil_spk = _ceiling_solid(round(cx - 19.0, 2), round(cx + 17.0, 2), round(cy - 20.0, 2),
                              round(cy + E.SPK_D / 2 + SPK_CLR + SPK_RAIL_T[1] + 2 * CEIL_PITCH, 2), round(SPK_Z_SEAT - 7.5, 2))
    # was_until_2026_09_25_hr43d: from .head import H03_ADP_Z
    # was_until_2026_09_25_hr43d: ceil_adp = _ceiling_solid(70.0, 83.0, -15.0, 15.0, round(H03_ADP_Z[1] + 0.2, 2))
    # was_until_2026_09_25_hr43d: ceil_tine = _ceiling_solid(70.5, 74.0, -15.0, 15.0, round(ADP_TINE["z0"] - 0.2, 2))
    # was_until_2026_09_25_hr43d: adds = [inter(b, ceil_spk) for b in speaker_mount_solids()] + [inter(b, ceil_adp) for b in adapter_press_solids()]
    # was_until_2026_09_25_hr43d: adds += [inter(b, ceil_tine) for b in adapter_tine_solids()]
    # hr43d（B′，agent_brief_hr43d 第 3 条）：转接板平放到 H08 托板上 → F06 压筋 ×2、F09 叉齿 ×2 删（adapter_press_solids / adapter_tine_solids 只留痕）
    adds = [inter(b, ceil_spk) for b in speaker_mount_solids()]
    adds += [inter(b, _shell_hull()) for b in keep_boss_solids()]
    plen = hr43_post_lengths(shell)
    f = H05_POSTS["fin"]
    for (x, y), post in zip(H05_POSTS["xy"], hr43_post_solids()):                                     # 每根柱（+ 它的筋）用自己那一小块内面截
        ceil_post = _ceiling_from(shell, round(x - f["L"] - 1.0, 2), round(x + f["L"] + 1.0, 2), round(y - 4.0, 2), round(y + 4.0, 2), H05_POSTS["z0"] - 0.5,
                                  wall=(1.1, 6.0))                                   # hr43c：窗口可能压到 F10′ 减薄格（竖直壁 ≥1.2）→ 下限 1.4 → 1.1
        adds += [inter(post, ceil_post)] + [inter(b, ceil_post) for b in hr43_fin_solids({(x, y): plen[(x, y)]}, only=(x, y), side=-1)]
        fin_plus = hr43_fin_solids({(x, y): plen[(x, y)]}, only=(x, y), side=1)
        if fin_plus:                                                                                      # +x 片自己一条窄内面（y ±1.0），长到 x + L_plus + 1
            ceil_fin = _ceiling_from(shell, round(x, 2), round(x + f["L_plus"] + 1.0, 2), round(y - 1.0, 2), round(y + 1.0, 2), H05_POSTS["z0"] - 0.5)
            adds += [inter(b, ceil_fin) for b in fin_plus]
    m = union(shell, *adds)
    # was_until_2026_09_25_hr43c: m = diff(m, *vent_cutters(), *sound_hole_cutters(), *hr43_soc_vent_cutters(), *hr43_sbc_relief_cutters())
    m = diff(m, *vent_cutters(), *sound_hole_cutters(), *hr43_soc_vent_cutters())   # hr43c：F14 板角让位删（板挪后 板↔原壳 1.284 @官方 R2.45 板角 ≥1.0，hr43c/c1b_board_r245.json）
    m = diff(m, *JAW.shell_pivot_cuts(slot=True), JAW.relief_sweep())
    m = diff(m, *JAW.h05_hub_slots(), *keep_pilot_cutters(), *hr43_pilot_cutters())
    m = diff(m, *h05_hr50_cuts())                                                    # hr50：毂盘让位 / 合缝唇留下的薄皮削穿（见 h05_hr50_cuts）
    return clean_print_topology(keep_main(m, "head_top_shell"))
