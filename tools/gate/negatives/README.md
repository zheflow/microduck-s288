# `tools/gate/negatives/` —— 反例回归包

元规则 8：**Gate 每次运行先跑这里，证明自己每个都能报红，否则 Gate 本身判失败。**
每条是项目真踩过的坑。坏样本一律用"对正常输入做一个最小突变"的方式生成：几何在内存里现造（trimesh 布尔出小网格 →
写进 `tools/gate/out/_neg_tmp/`，不进 git），判据一律取真实的 `data/*.yaml`，只把 parts / features / fasteners /
relations 换成这一条反例自己的最小清单。每个反例 < 5 s（L7 整机量允许 30 s）。

跑法：`./.venv/bin/python tools/gate/negatives/n_xxx.py`（单个）；`runner.py` 跑全包（≈7 min）。

## 结果契约（2026-09-13 起，`runner.check_contract` 校验；缺一项 = 失败）

以前只验 `state`：`min_wall_geometric` / `support_reachable` 降成 WARN 之后反例照过，"翻回 BLOCK 也没人抓"
真发生过（审计 F-反-1）。现在每个 `n_*.py` 的 `run()` 必须返回：

```
{"name", "passed", "expect", "got", "detail",
 "expect_severity": "BLOCK" | "WARN",      # 坏样本应该红在哪一级 —— 必须是层里**现在真实的**严重级
 "red": [record, ...]}                     # 坏样本上真正红了的判据，由 _harness.assert_red 给出
```

- `red` 非空；每条 `state == FAIL`、`severity == expect_severity`、`measured` 非 None（= 抓到缺陷，不是"算不出来"）。
- 专门验"坏声明必须 unknown"的反例（孔阵位置不全、NaN 镜像、区间反写…）另给
  `allow_unknown_red: "<为什么 measured 为空是设计>"`。
- 内核级反例（不经过层）用 `assertions: [{"claim", "ok"}, ...]` 代替 `red`，全 ok 才过。
  **注意 runner 先看 assertions：给了 assertions 就不再校验 red**，所以带已知缺口记录的文件
  （如 `n_l0_units_meters.py` 的 Y-up）把 `assert_red` 的结果也写成一条 assertion。
- 空壳（`passed=True` 但既无 red 也无 assertions）算失败。
- 对照样本用 `_harness.assert_green`（PASS 且 evidence_n > 0，ABSENT 不算绿）—— 判据不能写成恒红。
- 红必须来自层的 `run(ctx)`（或 run 直接调用的层入口，如 `run_declared_motions` / `run_tool_access`）
  发出的 finding，不是只测辅助函数 —— 函数改名/不再被 run 调用时反例要跟着红。
- 严重级只记录不改：层是 WARN 就写 WARN 并在 NAME/EXPECT 里标 `[WARN]`（目前：`min_wall_geometric`、
  `support_reachable`）。

`_harness.py` 提供：`FakeCtx`（含 `mjcf()`）、`base_data`、`box/cyl/export_stl`、`findings`、
`assert_red / assert_green / record / result`。

## 事故表（README 元规则 8 举的例子）

| # | 坏样本 | 怎么造 | 该报红的层 | 报红判据 | 文件名 |
|---|---|---|---|---|---|
| N1 | **空刀**（孔没切出来） | 声明 Ø2.2 通孔，成品上刀没落在实体上 | **2 特征存在** | `<孔>:present` FAIL(BLOCK)，measured "0/1 处到位"，证据数 = 射线数 > 0；对照有孔 PASS | `n_l3_bearing_section.py`（N1 段，走 `l2_features.run`）；孔中段缩径另见 `n_l2_fg07_bore_midspan_necking.py` |
| N2 | **轴承只查截面** | 把 6704 实体换成 Ø27×0.1 的薄片（只有中截面），座外侧留 0.2 mm 唇 | **3 静态装配** | 完整实体 × 座 → `bearing_6704zz:part_vs_component_solid` FAIL(BLOCK) ≈16.8 mm³ 且 `L01:component_bearing_6704zz` 红；薄片按 envelope 20×27×4 反算体积/包围盒**认领不到** → `world_placement` unknown（不许绿） | `n_l3_bearing_section.py` |
| N3 | **压板 0.30 均匀间隙夹不住** | H02 垫柱端面从 −14.85 退到 −15.15，清单仍写 contact=0 / status=ok / src=measured | **5 功能达成** | `relation_contact` FAIL(BLOCK)，有符号配合量 −0.30；贴合对照 PASS | `n_l5_fg13_pad_contact_label_only.py` |
| N4 | **扫掠区间取反** | frozen.yaml 某关节 `range_deg` 写成 `[92, -62]`（头尾对调）；`check_angles(lo>hi)` 只给两端点 | **6 运动** | 期望 `_joints`/该关节 有一条 FAIL(BLOCK)。**现状：层按上游 MJCF 区间扫，frozen 反写只进已退役的 `range_matches_upstream` → 没有任何 FAIL，本反例当前失败，等层在 lo>hi 时判 unknown** | `n_l6_reversed_range.py`（当前 ❌，待层修复） |
| N5 | **螺丝加长 2 mm** | fasteners.yaml F03 M2×6 改成 M2×8 | **5 功能达成** | `screw_length_rule`：总长 ≤ 叠厚 + 盲深 − 0.3，逐孔算 | 无（相近：`n_l5_fg14_shallowest_engagement.py` 验逐孔最浅咬入不按组取 max） |
| N6 | **删一个件** | 从导出集合里拿掉一个件 | **0 网格与坐标** + **4 装得进去** | L0 `stl_exists` 点名缺件；L4 引用该件的步无法构造 | 无（相近：`n_placed_instances.py` 只覆盖 L4 的 `_framework:placed_instances` 实例缺失/重复/跨件占用） |
| N7 | **单位错成米** | 导出 STL 整体缩放 0.001 | **0 网格与坐标** | `units_and_scale` FAIL(BLOCK)，measured 0.04 mm ∉ [3, 300] | `n_l0_units_meters.py` |
| N8 | **Y-up** | 导出前绕 X 转 −90° | **0 网格与坐标** | 舵盘轴心 vs MJCF ±0.05 / 基准轴夹角 ≤0.5°。**现状：`datum_vs_mjcf` 未实现、恒 unknown，包围盒判据抓不到朝向** —— 记录为已知缺口（assertions 里 ok=True 但写明），不假装红 | `n_l0_units_meters.py`（已知抓不到） |
| N9 | **镜像件只改左边** | 右件槽挪 1 mm（包围盒与圆孔中心都不变） | **2 特征存在** | `mirror:surface` FAIL(BLOCK) 1.0 > 0.05；真镜像 PASS；单向 NaN/Inf/空采样必须 unknown | `n_l2_fg09_mirror_rib_only.py`、`n_l2_nonfinite_mirror.py` |
| N10 | **舵盘时钟错 60°** | 把 `horn_holes` 旋转 60° | **5 功能达成（当前不可实现）** | ⚠️ 六孔等分阵列绕自身轴转 60° 是**几何自同构**，任何基于孔阵的判据都抓不到。要真正可检测必须引入非对称基准或用 S288 输出轴编码器零位在装配时标定。`features.yaml` 8 条 `horn_hole` 的 `clock_phase.declared` 恒为 null 且判 BLOCK 是**正确的**（未知=失败）。对应实机项 `bench.yaml:BN13` 第 2 条 | 无（不可几何判定） |

## 现有反例一览（按层）

| 层 | 文件 | 红在 |
|---|---|---|
| 内核 | `n_core_evidence` / `n_core_obligations` / `n_core_severity` / `n_core_retired` / `n_core_coverage_no_id` / `n_core_partial_inherit` / `n_core_verify` / `n_core_waiver` / `n_core_waiver_cell_only` / `n_core_neg_cache_key` / `n_core_keepout_layer_routing` / `n_l6l7_retired_not_stale` / `n_core_layer_error`（gate_guard 09-25：假层模块走真 gate.main —— 层抛 TypeError / 层进程被 SIGKILL / 返回不可解析 → `L<n>/_layer_error` BLOCK + 终端摘要与 md 顶部 ⚠ + 退出码 1；全量空跑加格、子集空跑只提示；干净 → CLEAR 无格） | assertions（build_scorecard / apply_waivers / verify / 反例缓存 key 静态全盖 + 运行时清单 / 禁入体义务按 checked_in_layer 派层 + covered_by_subjects 汇总格 / gate.main 层崩溃·空跑守门） |
| L0 | `n_l0_units_meters` | `units_and_scale` BLOCK；Y-up 已知缺口 |
| L1 | `n_l1_thin_wall` `[WARN]` / `n_l1_overhang` `[WARN]` / `n_l1_support_in_bore` / `n_l1_support_in_box_zone` / `n_l1_support_landing_tiers` `[WARN]` / `n_l1_single_bead_gcode` / `n_l1_missing_array_member`（unknown） / `n_l1_calibration_record` / `n_l1_feature_zone_snap`（hr41g：卡扣/舌片按 id 纳入 `no_support_zones.feature_zones`，面邻域外扩；未纳入/margin 缺或 0/tier 坏/无盒/无此特征 → unknown） | `min_wall_geometric` WARN / `support_reachable` WARN / `support_in_no_support_zone` BLOCK（真 G-code 落点，forbid 级） / `support_in_removable_zone` WARN（removable 级 + 后处理文本） / `min_wall_sliced` BLOCK |
| L2 | `n_l2_fg07_bore_midspan_necking` / `n_l2_fg08_plate_hollowed` / `n_l2_fg09_mirror_rib_only` / `n_l2_nonfinite_mirror`（unknown） | `bore_gauge` / `solid_retention`+`min_section` / `mirror:surface` |
| L3 | `n_l3_bearing_section` / `n_l3_ko01_independent` / `n_l3_ko01_neighbor_zone`（09-21：W 只查载体的两侧边界） / `n_l3_component_envelope` / `n_l3_frozen_items` / `n_l3_keepout_data_driven` / `n_l3_ko22_sweep`（hr41g：bounds_world_mm 包络沿方向后抽；障碍含元件；终点脱离；字段缺 unknown） | `part_vs_component_solid` + `component_<id>`；薄片 → `world_placement` unknown / `keepout_KO01`（声明加深通窗里的料，CAD 切刀看不见；pocket 模式侧出线槽里的料红、window 模式不查）+ dims 缺 → unknown / `part_vs_component_envelope` + `component_envelope_<id>`（占位体挖缺口 → 同源判据 0 绿、独立实体红）+ `envelope_solid_declared` unknown / `frozen_battery_bay_26:BAT_Y`·`frozen_h01_sbc_posts:hole_pitch`·`frozen_imu_pose_vs_cad` BLOCK + `measured_on` 缺 → unknown / `camera_fov_unobstructed`（光轴来自数据，缺 → unknown）·`keepout_intersection`（KO12 豁免只认 exempt[]）·`axis_joint_declared` unknown |
| L4 | `n_l4_split_direction` / `n_l4_zero_length_path` / `n_l4_segment_end_inside` / `n_l4_rigid_subassembly` / `n_l4_tool_far_seal` / `n_l4_tool_seat_origin` / `n_placed_instances` / `n_l4_present_derived` / `n_l4_elastic_declared_warn` `[WARN]` / `n_l4_elastic_outside_zone` / `n_l4_elastic_over_cap` / `n_l4_elastic_missing_decl`（hr41g：声明式弹性接触，共用 `_l4_elastic_scene.py`） / `n_l4_tool_cone_seat`（hr41 落盘 2026-09-25 Lane C：沉头锥面坐面 seats[].cone —— 锥口被料挡 / 锥口外有盖必红、好锥口绿、不写 cone 按平面判必红、cone 缺 src unknown；同轴异位两孔按声明孔深区分，挪 ≤ 孔深或不写孔深仍 unknown） | `step_sweep` / `disassembly_prereq` / `motion:*` / `tool_path_to_outside` / `tool_origin_on_seat` / `_framework:placed_instances` / `present_covers_derived` BLOCK + `present_only_installed` WARN + 缺子总成 unknown |
| L5 | `n_l5_fg13_pad_contact_label_only` / `n_l5_fg14_shallowest_engagement` / `n_l5_fg15_ninth_hole_sloped_seat`（+E 料里 unknown） / `n_l5_self_tap_radial` / `n_l5_fit_measured_only_on_pass` / `n_l5_head_on_pilot_hole` / `n_l5_seat_origin_cutter_mid` / `n_l5_seat_instance_key` / `n_l5_threshold_src` `[WARN]` / `n_l5_pilot_depth_priority` / `n_l5_joins_format`（gate_guard 第 6 项 09-25：joins 被 YAML 拆碎 → 只红该组 `joins_format` BLOCK、该组盲深不从文字取、层不崩、不出 `L5/_layer_error`） | `relation_contact` / `engagement_per_hole` / `self_tap_radial_overlap` / `fit_measured_geometry` / `head_seat_flatness`（坐面起点 = seats[].point_export_local，刀心只当轴锚）/ `head_side` unknown（同 instance 反向）/ `threshold_assumed` WARN + 无 src → `fit_value` unknown / `screw_length_rule`（pilot_depth_mm 优先，不一致 unknown）/ `joins_format` BLOCK(measured=`joins[3]=int 23`) |
| L6 | `n_l6_scene_inventory` / `n_l6_reversed_range`（unknown） / `n_l6_combo_pocket` / `n_l6_target_range_severity` `[WARN]` / `n_l6_target_range_declared`（unknown） / `n_l6_gen_range_vs_free` / `n_l6_undeclared_zero_gap` / `n_l6_harness_sweep`（hr44reg） | `_scene:placed_inventory`；`_joints:range_declared_ordered:*` unknown；`pair_combination` BLOCK（5° 网格抓两轴都非零的 5° 口袋，粗网格假绿）；`single_axis_sweep / pair_combination / collision_buckets / motion_collision` 目标区间外 → WARN、区间内 → BLOCK（assertions 双验）；目标区间声明缺/无序/非数/超域/收窄无 date+reason → 同组判据 unknown；`_ranges:collision_free_range` BLOCK（生成 MJCF range ⊄ 无碰撞区间）+ `_ranges:gen_range_covers_target` WARN（⊉ 目标区间）；`_clearance:min_clearance` BLOCK（零位间隙 0.2 没声明 contact 不豁免，真几何 min_gap 0.109）+ `_zero:undeclared_zero_gap_pair` WARN；`<线路>:harness_sweep` BLOCK（线束模型区内线∩件 / 零位弯 R < 限；调不通 / 替身场景 NOT_RUN(BLOCK)） |
| L7 | `n_l7_mass_torque` / `n_l7_inertia_tensor_all` / `n_l7_static_torque_home_and_range`（含 unknown） / `n_l7_mass_gap_unknown`（unknown） / `n_l7_component_claim` / `n_l7_torque_source` / `n_l7_generalized`（共用夹具 `_l7_fixture.py`：真数据下质量清单有缺口、整机判据一律 unknown，对照用"只删不编"补成完整清单） | `joint:*:static_torque` / `_stability:com_in_support_home` / `joint:*:forcerange_vs_servo_rating` / `_mjcf:inertia_tensor_parsed` / `joint:*:static_torque_home` BLOCK + 区间反写 → `static_torque` unknown / 清单缺一个质量 → `com_in_support_home`·`static_torque`·`head_mass_budget` unknown / `<元件>:component_qty` BLOCK(measured=实例数) + 无 claim → `component_body` unknown / `_load:torque_source_consistent` BLOCK(measured=差值) / `_mjcf:mirror_body_pairs` BLOCK(== 声明对数) + `_load:home_pose_declared`·`_stability:support_polygon`·`rib_thickness` unknown + `threshold_assumed:*` WARN |

补充要求：
- 每个坏样本除了"红"，还要断言**红在对的格子上**（`<件号>.<层>.<检查名>`），红错地方视同没抓到。
- 每条反例都要有"正常样本必须绿"的对照，防止判据写得过严把好件也判红。
- 新踩的坑一律先在这里加一条，再去修 Gate —— 顺序反了就没人知道 Gate 到底能不能抓。
- L6/L7 这类整机层跑不动最小解析件时，允许用替身 placed/（同名 1 mm 立方）或真 placed/ 只读 + 猴子补丁
  层的 `PLACED` / `_SIM_MJCF` / `_Scene`，但断言必须落在 run 发出的 finding 上，并在文件头写明停在哪一步。
- L1 支撑落点（2026-09-13 C-1，取代同日早先的 F-L1-3 过渡保护 + 两遍切片桩）：`support_in_no_support_zone` 按
  `slice_run.parts.<件>.support_landing`（真 G-code 采样点）判，没有记录 / source_sha256 或 zones_sha256 对不上一律 unknown。
  反例件没有真切片，用 `_harness.landing_stub(data, pid, stl, [支撑线段…])`：把"切片器在 export_local 这里放了支撑挤出线"
  写成合成 G-code（`;TYPE:Support material` + G1 E 段），按 slice_l1.main 同一变换转到打印坐标，再交给
  `slice_l1.support_landing` 真实解析 / 采样 / 逆变换 / 对 `l1_printable.no_support_zone_set(data)` 数点。桩只提供
  "支撑在哪"这一事实，不证明切片器会那么放；分级（forbid/removable）、阈值、sha 核对全在层里被检验。
  对照样本也要带桩（没支撑线的空记录），否则主判据是 unknown 不是绿（`n_l1_missing_array_member` 的好样本就是这样）。
  `mutate_zone_data=` 允许建桩**之后**改禁撑区声明，验"声明变了没重切 → unknown"（`n_l1_support_landing_tiers` B）。
- L6 反例共用 `_l6_scene.py` —— 替身 placed/ + 真运动树，`scene_class(keep=, hit_fn=)` 只留几只实体 / 用合成 evaluate 验网格与分档逻辑，`restrict_joints` 只扫几条关节，`gen_mjcf_with_ranges` 造临时"生成的 MJCF"，`run_l6` 猴子补丁 `PLACED/_Scene/_GEN_MJCF/_RANGES_OUT` 跑完还原，真 `out/mjcf_ranges.json` 不被反例覆盖。验真几何间隙的（`n_l6_undeclared_zero_gap`）不给 hit_fn，走真布尔。
- L7 整机层的对照样本必须先过 `_l7_fixture.complete_inventory`（2026-09-13 F-L7-4 起真数据的 static_torque / com_in_support_home / head_mass_budget 因质量清单缺口一律 unknown）；夹具只删不编（无实体无质量的元件 qty=0、无质量的轴承与 orig_* 连实体一起从替身 placed/ 拿掉、6704/6700 收成单一宿主件），是夹具不是对真数据的主张。
- L3 反例（2026-09-13）：KO01 用真运动学 `duckstructure.lib.sfw` 定舵机落位、件是造的；冻结项反例用造的仓/门/立柱/IMU 块 + 最小 frozen 字典（`measured_on` 指到反例件），阈值取真 `tolerances.yaml:frozen_dimension_mm`。
- 2026-09-26（hr46 E 段）退役 `n_l6_harness_sweep`：用户原话『注释掉吧，我已经开始打印了，后面我会再找你，有问题那个时候再修吧。』—— L6 E′ 线束实体模型扫掠调用已注释（l6_motion.py 文件头「E′ 停用」），本反例整文件注释并改名 `retired_n_l6_harness_sweep.py`（运行器只 glob `n_*.py`，不留恒绿反例）；恢复方法写在两个文件头。
