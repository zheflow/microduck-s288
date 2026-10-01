# duckstructure —— S288 版 Microduck 整鸭 CAD（按身体部位组织）

原来的 `tools/cad/duck.py`（1122 行单文件）按身体部位拆成了这个包。**纯搬运重构**：几何、常量、阈值、采样数、注释一字未改，
拆前/拆后 17 个 STL 的 sha256 完全一致、两个 json 去掉时间字段后相同、退出码相同。`tools/cad/s288.py`、`tools/cad/duck_kin.py` 也搬了进来
（`s288.py` 原样，`duck_kin.py` 改名 `kin.py`）。

## 怎么跑

```bash
# 在仓库根目录；一次只跑一个（4~6 分钟、吃内存）
./.venv/bin/python -m duckstructure.build
```

等价于原来的 `python tools/cad/duck.py`：导出 17 个 STL 到 `cad/duck_s288/`，跑零位干涉 + 逐关节扫掠 + 装配/机械审计，
写 `audit_summary.json` / `mechanical_audit.json`，`local/ placed/ scene_*/ xx/` 一并重出；有干涉或未闭环接口时退出码 **2**（当前预期就是 2）。

快速子集检查仍是 `./.venv/bin/python tools/cad/qcheck.py head|trunk|all`，装配抽出检查 `tools/cad/asmcheck.py`。

```python
import duckstructure as D          # 所有公开名字都从包顶层可拿：D.P / D.sfw / D.build_trunk / D.run_checks …
D.PARTS                            # 件号 → (STL 文件名, 导出用 MJCF 连杆, build 函数)，顺序 = 导出顺序
meshes = D.build_all()             # {件号: 世界坐标网格}，只建不导出不检查
```

## 改几何只改这里

| 想改什么 | 改哪 |
|---|---|
| 舵机尺寸（标 `[量]` 的实测值） | `s288.py` 里的 `S` |
| 关节轴线 / 舵机位姿 | **不要改** —— `kin.py` 从 `upstream/microduck_rl/.../robot_walk.xml` 读，是训练好的策略骨架 |
| 参数表 `P`、轴承/背板/踝副轴常量、电池仓位置（`BX0/BX1/BZ0/BZ1/RAIL_Y…`）、壳安装柱 `SHELL_BOSS`、`SERVO_SHIFT` | `lib.py` 顶部（冻结项，动之前先看 `docs/`） |
| 某一个件的形状 | 对应部位模块里的 `build_*`（下表） |
| 检查阈值 / 扫掠范围 `SWEEP` / 旧检查域 `LEGACY_LIMITS` | `checks.py` |
| 导出顺序、哪些原版件参检、场景导出 | `build.py` |

## 17 件一览

| 件号 | STL | build 函数 | 模块 | 装什么 | 依赖哪些别的件 |
|---|---|---|---|---|---|
| L01 | `L01_yaw2roll.stl` | `build_yaw2roll()` | legs | 髋偏航从动盘 + 髋横滚舵机载体（照原版 yaw2roll 重雕） | L02（左右髋件绕横滚/偏航轴扫掠让位） |
| L02 | `L02_hip.stl` | `build_hip()` @memo | legs | 纯双从动盘：一面髋横滚、一面髋俯仰 | — |
| L03 | `L03_upper_leg.stl` | `build_upper_leg()` @memo | legs | 大腿：髋俯仰 + 膝两颗舵机的等厚背板 + 膝 6704 座 | — |
| L04 | `L04_lower_leg.stl` | `build_lower_leg()` @memo | legs | 小腿：膝从动盘 + 踝舵机背板 + Ø9.95 踝副轴颈 | L03（膝反向扫掠）、电池实体（髋偏航扫掠） |
| L05 | `L05_ankle_foot.stl` | `build_ankle_foot()[0]` ← `build_ankle_parts()` @memo | legs | 踝主脚（法兰侧从动盘 + 脚板） | L04（踝反向扫掠 ±`ANK_SWEEP`）；`tools/cad/ankle_split.py` 分体 |
| L06 | `L06_sole_TPU.stl` | `build_ankle_foot()[1]` | legs | TPU 鞋底（原版 sole 减去 L05/L07） | L05/L07 |
| L07 | `L07_ankle_rear_arm.stl` | `build_ankle_rear_arm()` | legs | 可拆后轴承臂（6700ZZ 座，M2×12 接 L05） | 同 L05 |
| T01 | `T01_trunk.stl` | `build_trunk()` | trunk | 躯干：两个髋偏航载体 + 顶板 + 颈俯仰轭 + 电池仓 + 原版腹面甲板 + 壳柱 | 原版 left/right_shell（当刀）；`battery_bay()` |
| B01 | `B01_battery_door.stl` | `build_battery_door()` | trunk | 电池仓门（底脚 + 卡珠槽 + 2×M2） | — |
| T02 | `T02_shell_L.stl` | `build_trunk_shell(+1, trunk)` | trunk | 左躯干壳（原版外形，挖脖子活动空间 + 2 个 Ø2.4 孔） | T01（颈部区让位）、N01+N02（`neck_swing_env`） |
| T03 | `T03_shell_R.stl` | `build_trunk_shell(-1, trunk)` | trunk | 右躯干壳 | 同 T02 |
| N01 | `N01_neck.stl` | `build_neck()` @memo | neck | 脖子：颈俯仰 + 头俯仰两颗舵机背靠背的载体 | N02（头俯仰 ±45° 扫掠） |
| N02 | `N02_neck_pitch.stl` | `build_neck_pitch()` @memo | neck | 颈俯仰件（U 形轭 + 偏航立筒，照原版重雕） | — |
| N03 | `N03_yaw_roll.stl` | `build_yrm()` | neck | 偏航-横滚件（A/B 座盘 + 等厚背板 + 裙墙） | — |
| H01 | `H01_head_bracket.stl` | `build_head_bracket()` | head | 头内支架（原版 motor_support 毛坯 + 横滚舵机笼 + 香橙派立柱） | N03（横滚 ±25° 扫掠） |
| H02 | `H02_head_clamp.stl` | `build_head_clamp()` | head | 可拆压板（两根垫柱压横滚舵机背孔，两颗 M2×10 接 H01） | — |
| H03 | `H03_head_bottom_shell.stl` | `build_head_bottom_shell()` | head | 原版下头壳的 S288 适配件（只挖舵机让位） | — |

右腿 5 件不单独建：`build.py` 用 `mirror_y()` 镜像左腿参检（`MIRROR` 表给出镜像后的 MJCF 连杆名）。

`@memo`（`lib.py`）：同一进程内同名 build 函数只算一次，结果放在 `lib._MEMO`。L01 里扫 L02、L04 里扫 L03、L05 里扫 L04、
N01 里扫 N02、T02/T03 里建 N01/N02 都走这个缓存，所以谁先建都不会重复算。

## 模块与依赖

```
s288 ───┐
kin  ───┼──▶ lib ──┬──▶ legs ──────────────────────┐
                   ├──▶ neck ──┬──▶ trunk ─────────┤
                   │           └──▶ head ──────────┼──▶ build（入口）
                   └──▶ checks ────────────────────┘
                                   __init__ 再导出以上全部公开名字，给 build.py / tools/cad 的检查脚本当"duck 模块"用
```

| 模块 | 放什么 | 行数 |
|---|---|---|
| `s288.py` | S288 舵机参数表 `S` 与舵机坐标系原语（`cyl/bx/union/diff/inter/placed/frame/servo_envelope/servo_mesh/horn_holes/mount_holes…`） | 166 |
| `kin.py` | 读 MJCF `robot_walk.xml`：`load()` → `B, ORDER`（连杆树、关节、舵机位姿、原版网格）；`mesh(name)` | 62 |
| `lib.py` | `OUT`、`B/ORDER`、参数表 `P`、全部全局常量（`XR/PLT/RING_T/ANK_*/电池仓/RINGS/SHELL_*/SERVO_SHIFT/XLF`）、`memo`、世界/舵机坐标工具（`TW/orig/sfw/drv_from/drv_self/pt/servo_env/to_local/mirror_y/MIRROR`）、舵机坐标原语（`carrier/spacer_card/card_cut/mount_cut/driven/driven_patch/horn_cut/horn_cbore/flange_relief/flange_slot/mnt_cut2/back_pad/back_shell/wide_y/brg_ring/ring_boss/bearing_driven_clearance/servo_slide`）、通用几何（`hull/wbox/ybox/fill_cyl/silhouette/sweep_of/keep_main/minkowski_box/clean_print_topology/stl_stats`）、原版壳（`shell/shell_inner_z/shell_cutter`） | 448 |
| `legs.py` | L01–L07 | 124 |
| `neck.py` | N01–N03 及其局部常量（`NK_*/NP_*/YRM_*`） | 132 |
| `trunk.py` | 电池仓（`battery_bay/battery_bay_cuts`，仓门/底脚/卡珠常量 `FOOT_*/DOOR_SCREW_X/SNAP_*`）、B01、`DECK_TOP`、T01、`neck_swing_env`、T02/T03 | 158 |
| `head.py` | `HEAD_CLAMP_MOUNTS`、H01–H03 | 88 |
| `checks.py` | `check_angles`、`EXPORT_STATS/export`（STL 闭合/单实体断言）、`vol`、`descendants`、`LEGACY_LIMITS`、`SWEEP`、`run_checks`（零位干涉 + 逐关节扫掠） | 109 |
| `__init__.py` | 再导出各子模块的公开名字、`PARTS`（件号→build 函数的有序表）、`build_all()`；把 `tools/cad/` 塞进 `sys.path` | 57 |
| `build.py` | 原 `duck.py` 的 `__main__` 原样：建 17 件 → 镜像右腿 → 原版头壳/舵机/轴承参检 → `run_checks` → `mechanical_audit.verify_mechanics` → 写两个 json → `local/ scene_*/ placed/ assembly_preview.stl` → `sys.exit(2)` | 111 |

归属原则：被谁用就放谁那，多处共用放 `lib.py`。和"部位"直觉不一致的几处：

- **电池仓位置常量**（`BAT_CLR/BX0/BX1/BZ0/BZ1/BAT_Y/RAIL_Y/RAIL_Z0/DOOR_*/BAY_FW/BRACE_Z/BZR0/BZR1`）在 `lib.py` 而不是 `trunk.py`：
  L04 小腿要拿电池实体绕髋偏航扫掠削让位，`build.py`/`qcheck.py` 也要放电池实体参检。仓门自己的常量（`FOOT_*/DOOR_SCREW_X/SNAP_*`）留在 `trunk.py`。
- `neck_swing_env()` 在 `trunk.py`：它建的是脖子的扫掠包络，但只有躯干壳用它挖料。
- `ybox()`、`mirror_y()/to_local()/MIRROR` 在 `lib.py`：L01 要镜像对侧髋件，`build.py`/`qcheck.py` 要镜像整条右腿。
- `check_angles()` 只被 `run_checks` 用，放 `checks.py`。

## 与 tools/cad/ 检查脚本的关系

`tools/cad/{assembly_audit,mechanical_audit,ankle_split,asmcheck,qcheck}.py` 留在原地，改成从 `duckstructure` 导入。
`assembly_audit.bearing_specs(D)` / `mechanical_audit.verify_mechanics(D, allm)` 需要一个带 `B/sfw/TW/cyl/vol/placed/s288/orig/inter/diff/…` 属性的"duck 模块"，
`build.py` 传的是 `duckstructure` 包本身（原来是 `sys.modules[__name__]`）。

包里保留了原文的三处**函数内延迟 import**（`lib.minkowski_box/clean_print_topology` → `assembly_audit.solid`，`legs.build_ankle_parts` → `ankle_split.split_ankle`），
它们 import 的是 `tools/cad/` 的顶层模块而不是包内模块，本身没有循环；`__init__.py` 把 `tools/cad/` 塞进 `sys.path`（原 `duck.py` 第 7 行的等价物）。
包内模块之间没有循环依赖，模块划分本身就是无环的 —— 没有任何一处包内依赖是靠函数内 import 掩盖的（上面三处延迟 import 指向的都是包外的 `tools/cad/` 脚本，原文就这么写）。

## 拆分时唯一改过的三处（都不是几何）

1. `lib.py`：`OUT = …("..", "cad", "duck_s288")`——原来是 `("..", "..", …)`，因为文件从 `tools/cad/` 上移了一层。`kin.py` 里 `MD` 同理少一个 `".."`。
2. `build.py`：`bearing_specs(sys.modules[__name__])` → `bearing_specs(D)`；`verify_mechanics(sys.modules[__name__], allm)` → `verify_mechanics(D, allm)`，`D` 即 `duckstructure` 包。
