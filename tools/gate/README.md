# Gate —— 可打印 / 可装配评价体系（规格）

> 目标：**换件（摄像头、麦克风、头壳……）之后不改检查器，还能查对。**
> 做法：检查规则只写**不变量**，不含任何具体零件的具体坐标；一切"应该是什么"都在 `data/` 里声明，换件只改数据。

状态：规格阶段。元件库与三份清单正在从原版 + V2 盘点生成（见 `docs/gate/inventory_*.{json,md}`）。

---

## 1. 数据（`tools/gate/data/`）—— 换件只改这里

| 文件 | 内容 | 一条记录长什么样 |
|---|---|---|
| `components.yaml` | **元件库**：每个外购件一条 | envelope / mount 孔阵 / connectors(位置·插头实体·拔插方向·线出口) / cable(根数·径·最小弯折半径·长度) / keepout(视锥·声孔·气路) / mass·com / power·heat / insert_dir / **每个字段标 measured\|datasheet\|assumed** |
| `parts.yaml` | 17 个打印件 | 由哪个 build 函数出、材料、打印朝向、镜像关系、houses(装了哪些元件) |
| `features.yaml` | **特征清单** | 每个孔/沉头/槽/凸台/夹持面：件、种类、名义尺寸、深度、通盲、方向、作用（固定谁）、连接类型 |
| `fasteners.yaml` | 每颗螺丝 | 规格、长度、连什么和什么、拧进什么材料、**它自己的刀具包络**（Ø4/Ø5/PH0…）、驱动方向 |
| `assembly.yaml` | **装配顺序清单** | 逐步：动哪个件、沿什么方向/路径、此时已装了什么、用什么工具；拆卸序也写 |
| `relations.yaml` | **功能关系清单** | 夹持/压配/滑配/轴承座/舵盘耦合/从动盘/铰链：谁和谁、目标量（过盈·间隙·同心）、依据（实测试件 or 占位） |
| `keepouts.yaml` | 禁入体 | 视锥、声孔、舵盘摆动扇区、电池抽出路径、SD/USB/开关/充电口、刀路、线束弯折空间 |
| `harness.yaml` | 线束路径 | 每束：折线 + 直径 + 最小弯折半径 + 两端插头 + 长度预算 + 跨哪个关节 |
| `frozen.yaml` | 不许动的 + **能力包络** | 关节轴线、电池仓 26 项、SHELL_BOSS、立柱、6 个原版 STL、**躯干 IMU 位姿**；`capability_envelope`（2026-09-16）：原版策略在 in-scope 任务模式下实际用到的关节角（来源 policy_envelope.json、左右镜像求并、margin），`joint_axes[].target_range_deg` 由 `derive_target_ranges.py` 从它推导——**判据来自原版能力，件去满足它**，不按件能转多少填 |
| `tolerances.yaml` | 公差表 | 按 **材料 × 打印方向 × 特征类型** 存；未实测的填占位值并标 `assumed`。**连接类型决定咬入范围**：S288 金属螺纹 1.8~3.0 / PLA 自攻 ≥2×d / 热熔螺母按螺母长 |
| `waivers.yaml` | 豁免 | 格子 id、理由、签发人、日期、**绑定的输入 hash**（输入一变自动失效） |
| `bench.yaml` | 实机验证清单 | 只能实测的判据（预紧变形、温升、掉线行为、疲劳…），现在写死数字，到货逐项打勾 |

---

## 2. 层（每层一句话判据，全部 mm / mm³，不四舍五入，未知 = 失败）

| 层 | 问 | 判据 | 数据来源 |
|---|---|---|---|
| **0 网格与坐标** | 是合法实体、在对的坐标系里吗 | 洞边=0 · 实体=1 · 退化面=0 · 非流形边=0 · **基准特征（舵盘轴心）落在 MJCF 位置 ±0.05** · 包围盒在预期范围（抓米/毫米、Y-up、未 apply transform） | parts / frozen |
| **1 可打印** | 打印机吐得出来吗 | 真切片：最薄壁 ≥2 圈 · 最小孔 ≥2×喷嘴 · 悬垂 >55° 的面在支撑可拆区 · **支撑落点按真 G-code**：forbid 区（轴承座/轴颈/锪平/走廊配合面壳层）= 0 点 BLOCK、removable 区 WARN + 后处理 · TPU 单独规则 | parts(朝向·材料) + printability(parts/no_support_zones) + tolerances |
| **2 特征存在** | 声明的孔/台/槽真的在吗 | 逐条：射线验通/盲深 ±0.1 · 直径 ±0.1 · 沉头 · 孔周壁厚 ≥ 阈值 · **镜像件特征镜像误差 <0.05** | features |
| **3 静态装配** | 零位姿各就各位吗 | 打印件 × 打印件、打印件 × **元件完整实体**（placed 占位体 + `envelope_solid` 独立实体两份）、× **禁入体**（几何/豁免全在 keepouts.yaml，KO01 不借 CAD 切刀）：交集 < 0.05 mm³ · **冻结项实测**（电池仓 26 项 / H01 立柱 / IMU 安装孔 ±0.1） | components / keepouts / frozen / fasteners ；**hr50（09-28）**：`frozen.yaml:battery_bay_26.door_plate_retired{probes,date,decided_by,regression_now}` 列出的门板侧探针（DOOR_T / BX0 / DOOR_YLO…）照量照记数字但发 RETIRED（用户 09-22 解冻电池仓+门、hr48 有意改门；回归转 L2 B01-F01）；非门板探针名 / 缺字段 → unknown 且不退役 |
| **4 装得进去** | 每个东西有路进到位吗 | 按 assembly.yaml 逐步：移动件沿路径扫掠 vs 已装件 = 0 · 每颗螺丝按**自己的**刀具包络有直线通道 · 螺丝头沿刀轴可见 · 拆序也通 | assembly / fasteners |
| **5 功能达成** | 装上后它干活吗 | 夹持过盈 · 卡扣咬合深 · **咬入按连接类型** · 螺丝总长 ≤ 叠厚+盲深−0.3 · 轴承配合按公差表 · 舵盘 6 孔同心 <0.05 · **舵盘时钟**（零位下孔阵相位 = 声明值，抓 60° 倍数错位）| relations / fasteners / tolerances |
| **6 运动** | 动起来撞不撞卡不卡、**原版能做的姿态我们做不做得出** | 单轴 2.5° 独立网格 + **相邻关节两两组合 5° 统一网格**；碰撞在 `frozen.yaml:target_range_deg`（由能力包络推导）内 BLOCK、外 WARN（工作空间比原版窄但原版任务用不到）；**能力判据** `_capability/*`：每个 in-scope mode 的包络 ± margin ⊆ 本层量出的无碰撞区间（BLOCK）、**原版策略真实姿态**（`policy_envelope.py --dump-steps` 逐步元组 → `policy_pose_collisions.py` 离线布尔 → `policy_pose_compare.py` 与原版全件同姿态逐 body 对比，summary/compare 双指纹核对）**hr50（09-28 用户定：极限姿势靠重训限制、不削）起按 `tools/gate/data/real_pose_policy.yaml` 判**：极限段（alpha_stand/body_all_max、roulade、alpha_sitstand/head_all_min）里的碰撞不阻断，逐对列进 `real_pose_retrain_constraints`（INFO：件对、姿态数、最大交集、最大那帧穿插深度、到头的关节）；其余姿态交集 ≤2.2868 mm³ 且穿插深度 ≤0.7696 mm（原版自身真接触峰值 p25，hr50 C 标定）为轻碰，超出或深度算不出 → `real_pose_collisions` BLOCK；原版共有碰撞仍不豁免；**相邻两关节组合 `pair_combination`（hr50 v3，09-28 用户定，同一 yaml 的 `pair_combination` 段）**：碰撞记录逐条分类 —— 有一轴在目标区间外 → 工作空间 WARN（F-L6-3 不变）；区间内、网格姿态 ±5.5°（网格半步 2.5 + margin_deg 3）内原版策略逐步记录（`frozen.yaml:capability_envelope.pose_steps_source`，sha 校验，去掉三段极限段，按 mirror_pairs 左右镜像求并）从没到过 → 进 `pair_combination_retrain_constraints`（INFO：件对、最大交集、深度、撞点网格按连通块给耦合区间『q1、q2 不能同时进这个区间』，各向外扩一格）；到过 → 同上轻碰口径，超出 / 深度算不出 BLOCK；**v4（09-28 用户定）**：超轻碰但落在 `approved_retrain_constraints`（用户逐条批准的重训联动限位：件对 × 关节对 × 耦合盒 × 体积上限，上限 = 正式 chain 在冻结 STL 上量的数、不留余量）盒内且 ≤ 上限 → 不 FAIL、逐条列出，超上限 / 盒外照 BLOCK，登记了没命中的列 unused；重训约束另按关节对合并成训练用区间（`per_joint_pair`）；**v6（09-28 用户定 C）**：`_policy/policy_extreme_poses` 的策略包络角点姿态记录（每 mode 30 个 14 轴同时到包络角的姿态，不是可达集）也按同一口径逐条分类（『到过』= **全部 14 轴**同时 ±5.5° 内有原版样本），没到过 → `_policy/policy_extreme_poses_retrain_constraints`（INFO），只有『到过且超轻碰』BLOCK、只有区间外 WARN；件格子 / 汇总按 `pose_class` 同分类；口径段缺 → unknown；同一批记录在 `motion_collision` / `collision_buckets` 里也按这个分类，只有『到过且超轻碰』算件的问题；包络证据坏 / 分类失败 → unknown；能力盒内的碰撞只 WARN（轴对齐盒是上界，README A2）；生成 MJCF range ⊆ 无碰撞区间、⊇ 目标区间；运动对**最小距离 ≥ 公差表值**（豁免只认 relations 声明的 contact）；线束沿路径扫管过全行程 | frozen(capability_envelope, target_range_deg) / policy_envelope.json / relations(contact) / harness / tolerances |
| **7 质量与力** | 撑得住、站得稳吗 | 17 件体积×密度 + 元件实重（按 `components.yaml:claim` 认领）→ 每 body 质量重心 · **与 MJCF `<inertial>` 逐 body 比漂移** · 每关节扫描峰值与 home 姿态静扭矩 < S288 额定 × 系数 · 零位/home 重心投影在支撑多边形内 · **质量清单有缺口 → 整机判据 unknown（只给下界文本）** · 承力最小截面 / 薄筋长厚比 / 层向 vs 受力 | components(mass, claim) / parts(ground_contact) / frozen / tolerances(load) |

不进 Gate、进 `bench.yaml` 的：预紧变形与摩擦、多舵机峰值电流与压降、温升、掉线/重上电行为、线束疲劳、热管理、传感器实际可用性、耐久。

---

## 3. 元规则（保证"绿"是真绿）

1. **查最终导出的 STL 读回来的东西**，不查内存网格。送去打印的是文件。
2. **每个 0 带证据数量**：几对、几个角度、几条射线。`0 碰撞 / 0 对` = 没跑，不是通过。
3. **采样网格独立于建模角度**（`check_angles()`）。
4. **未知 = 失败**：布尔炸了、特征找不到、清单没写、字段是 `assumed` 而配合间隙 < 公差表值 → 都红。
5. **不变量，不比历史**：体积/交集与上一版的差异只做 WARN 级"变更告警"触发人看，**不是放行条件**。
6. **hash 绑定**：记分卡写入 `duck.py`、`s288.py`、`robot_walk.xml` 的 git hash + 17 个 STL 的 sha256 + `data/*.yaml` 的 sha256。hash 对不上的记分卡作废；发布副本 ≠ 源码直接红。
7. **过期标记**：每个格子记录输入 hash；输入变了格子变灰，不得沿用旧绿。改一个件只重跑受影响子集。
8. **反例回归包先跑**：`negatives/` 里每个历史事故一个坏样本（空刀、轴承截面、压板零间隙、扫掠区间反、螺丝加长 2 mm、删一个件、单位错成米…）。Gate 每次运行先证明自己**每个都能红**，否则 Gate 本身判失败。
9. **三级 + 豁免**：BLOCK / WARN / INFO。只有 WARN 可豁免，豁免绑定输入 hash，输入一变自动失效。
10. **性能**：内容 hash 缓存 · AABB / 采样距离粗筛后再精确布尔 · 单件检查可独立跑；整机全绿一次不应超过现在 `duck.py` 的一半时间。

### 3.1 元规则落地状态（2026-09-13 审计后第一批；每条修复都有反例）

| # | 状态 | 落在哪 | 反例 |
|---|---|---|---|
| 1 | 落实 | 各层读 `cad/duck_s288/*.stl` / `placed/*.stl` | n_l0_units_meters / n_l2_* |
| 2 | 落实 | `core.build_scorecard`：evidence_n=0 的 PASS → NOT_RUN | n_core_evidence |
| 3 | 落实 | `duckstructure/checks.py:check_angles` 零度对齐网格 | n_l6_reversed_range |
| 4 | 落实（框架级补丁） | 覆盖率义务在清单缺 id 时**写红格**而不是跳过（`core.py:COVERAGE_ITEMS` 循环） | n_core_coverage_no_id |
| 5 | 落实 | 层不读 prev；`impact.md` 只报变化 | — |
| 6 | 落实 | `gate.py --verify`：逐格 inputs_hash + source_manifest + git HEAD 与磁盘核对，不一致退出码 3 | n_core_verify |
| 7 | 落实 | 格级 inputs 隐含 `duckstructure/*.py`；`--layers/--parts` 子集跑写 `scorecard.partial.json`、不写 prev，范围外格子按 hash 继承 | n_core_partial_inherit |
| 8 | 落实（契约收紧） | 反例必须声明 `expect_severity` 且 `red` 里每条 measured 非 None（`negatives/runner.py:check_contract`）；结果按 `sha256(negatives+layers+core+gate+tool_access+slice_l1+data)` 缓存到 `out/negatives.cache.json` | 全部 n_* |
| 9 | 落实 | 豁免 id 必须带判据名；`bound_input_hashes` = {仓库相对路径: sha256}（`waivers.yaml:schema`） | n_core_waiver / n_core_waiver_cell_only |
| 10 | 部分 | 反例缓存命中 ≈ 0.03 s；层级结果缓存仍未做（全跑 ≈ 30 min；09-13 第二批后 L6 组合 5° 网格多 ≈5 min） | — |
| 4b | 落实（L5/L7） | **阈值也要有来源**：`fits.*.target_range_*` / `feature_check_tolerances.*` / L7 密度·稳定裕度·筋容差 节点无 `src` → 引用它的判据 unknown；`src=assumed` → 照判 + `threshold_assumed:<键>` FAIL(WARN)，measured=阈值本身（`l5_screwhead.threshold_meta / AssumedThresholds`，l5_function、l7_mass 同源） | n_l5_threshold_src、n_l7_generalized |
| 4c | 落实（gate_guard 09-25） | **层崩溃 / 层空跑不许静默**：层模块抛异常 / 层进程被杀、进程池断 / 返回不可解析 / import 失败 → `L<n>/_layer_error`（FAIL，BLOCK，`layer_crashed`）；全量跑时层模块 0 条判据 → 同格（`layer_zero_findings`），子集跑只打印提示；终端摘要顶部与记分卡 md 标题下各一行 ⚠，结论 BLOCKED、退出码 1（`gate.py:_layer_guard`；断池时 `_exit_after_main` 用 os._exit 收尾防解释器收尾挂死） | n_core_layer_error |
| 8b | 落实 | 反例缓存 key **静态全盖**（`tools/gate/**/*.py`、`duckstructure/*.py`、`tools/cad/*.py`、`data/*.yaml`、`slicing/*.yaml`、`l1_calibration.yaml`、标定网格）+ **运行时** `sys.modules` 仓库内模块清单 + trimesh/manifold3d/numpy/shapely/Python 版本；命中时 `L0/_negatives` 照写、标 `cached` | n_core_neg_cache_key |

状态语义：`STALE` = 输入变了没重跑（继承可恢复）；`RETIRED` = 判据基准失去权威、只留记录（不阻断、不计 INCOMPLETE、单独计数，`core.py:RETIRED`）。层里**不许**再硬编码 `state=STALE`。

### 3.2 本轮新增的写层约束（2026-09-13，各有反例）

| 约束 | 来源 | 反例 |
|---|---|---|
| **覆盖 = 量到，不是发过**：`res.covered` 只登记至少发出一条带 measured 判据的条目；只发 unknown 的不算覆盖 | F-L2-1 | n_l2_coverage_gap |
| **量的是零件不是三角化**：从网格取尺寸必须对分段数免疫（棱边/顶点法），射线内接径只作旁证；float32 表示噪声（≤1e-5）可吸收，阈值不动 | F-L2-2 | n_l2_facet_diameter |
| **"有实测撑着" = 实测通过了**：桶/汇总格只能引用 state==PASS 的实测判据；量了没过要传红，不传"已量" | F-L5-3 | n_l5_fit_measured_only_on_pass |
| **方向必须声明，不取较差端**：头侧、压紧方向只从 data 的显式字段取，取不到判 unknown；"两端取较差"是把对面的缺陷记到这一面 | F-L5-1/2 | n_l5_head_on_pilot_hole、n_l5_fg15 |
| **没评过 ≠ 没误伤**：标定/统计类判据分母只数真评过的（PASS 且 evidence_n=0 不算）；范围内任一 BLOCK 判据一次没评 → 标定不成立 | F-L1-1 | n_l1_calibration_record |
| **"每个"就写 `== len`**：criterion 写"每个"的判据不许用 `if n:` 判 | F-L7-2 | n_l7_inertia_tensor_all |
| **区间必须有序**：任何 [lo, hi] 声明 lo ≥ hi 一律 unknown，不许让采样函数退化成两端点 | N4 | n_l6_reversed_range |
| **反例先跑一次红**：新判据/修复先写反例并在改代码前跑出失败，再改 | 元规则 8 | 本轮全部 |
| L4 工位在场全集必须 ⊇ 从 `seq`/`subassembly_of`/`subassemblies` 推导的最小集合（`present_covers_derived` BLOCK）；反向顺序矛盾 `present_only_installed` WARN；`tool_states` 必须带 `after_step`/`side` | F-L4-1 | n_l4_present_derived |
| L4 轴承 token（6700/6702/6704/`22×16×4`）→ 外径按 `components.yaml` 解析，不写死 | F-L4-1 / §2.6 | n_l4_present_derived（'6704×2' 类 token 进 parts） |
| **禁入体不借 CAD 切刀**：KO01 插头顶/线弯区 A + 侧出线槽 B / 背板通窗 W 由 `keepouts.yaml:KO01.dims` + `availability_table.rows[].mode_*` 独立建（09-20 背插模型），舵机落位只借 `sfw`；CAD `conn_cut` 只作对照 INFO（`keepout_KO01_vs_cad_cutter`）；dims 缺 → unknown | F-L3-2 | n_l3_ko01_independent |
| **元件实体要有一份不与 CAD 同源的**：`components.yaml:envelope_solid`（ring/box/none）独立建实体、落位只借 placed 的 OBB；`placed_covers_envelope`（顶点法，对分段数免疫）+ `part_vs_component_envelope`；同源的 `part_vs_component_solid` criterion 写明"与 CAD 同源"；未声明 → unknown | F-L3-3 | n_l3_component_envelope |
| **冻结项要真核**：`frozen.yaml` 的 `battery_bay_26`（T01/B01 射线）、`h01_sbc_posts`（H01 截面闭环）、`imu_pose`（T01 射线扇 + 生成 MJCF site + delta 算术）按 `measured_on` 对导出件实测，阈值 `frozen_dimension_mm`；射线分不出的（卡珠埋在导轨里）判 unknown，不是导出件几何的逐条 INFO 登记 | F-数-1 | n_l3_frozen_items |
| **禁入体参数与豁免全在数据**：KO06 `servo_frame_of_joint`、KO07 `axis_joint`、KO11 `probe_zone_z_mm`、KO12 `exempt[]`、KO15 `fov_deg/apex_world_mm/optical_axis_world` —— 缺 → unknown，不再默认 `neck_pitch/head_yaw/100..175/"B01"/+x`；元件认领按 `components.yaml:claim`（与 L7 同一份声明） | §2.6（L3） | n_l3_keepout_data_driven |
| **扫掠/运动类禁入体的义务派给真查它的层**：`keepouts.yaml:KOxx.checked_in_layer` + `covered_by_subjects`；`core.obligations` 按它派层，`core.build_scorecard` 把所列格子汇总成 `L{层}/KOxx`（缺格 NOT_RUN、红传红），第 3 层只留 NOT_RUN INFO 留痕 | L3 MINOR（`_DEFERRED` 永久 INCOMPLETE） | n_core_keepout_layer_routing |
| **支撑落点按真 G-code**：解析 `;TYPE:Support material/interface` 挤出段 0.2 mm 采样、逆变换回 export_local、对 `no_support_zone_set` 逐区数点；两级 forbid（BLOCK）/ removable（WARN + 后处理进打印清单）；`per_feature_overrides` 可改级；zones_sha256 只带**本件**特征的 override，别件改 override 不作废本件记录 | F-L1-3 / C-10..13 | n_l1_support_landing_tiers、n_l1_support_in_bore/_box_zone |
| **切片配置不进代码**：逐件朝向 / 填充率 / 周界生成器 / 追加参数全在 `printability.yaml:parts.<件>`（`down_world` 等），缺 → `slice_l1.py` 退出不给缺省；`--parts` 子集重切沿用其余件记录 | §2.6（slice_l1） | —（脚本级，层由 slice_freshness/zones_sha256 兜住） |
| **刀心不是坐面**：`features.yaml:hole_positions_mm` / `feature_hole_map[].holes` 是切刀中点（常离件 15–19 mm 甚至在件外），只能当轴线锚；头/坐面判据的起点一律是 `fasteners.yaml:tool_access[].seats[].point_export_local`，头侧 = `outward_export_local`；坐面点到孔轴横向偏差 > 1e-6 → unknown（与 `tool_access.py` 同规则）；孔轴优先取特征声明轴（`geom.instances[k]` 按 `feature_hole_map[].instance` 取），实测轴起点在孔腔里、只作 `hole_axis_declared_vs_measured` 旁证 | F-L5-2 第二半 | n_l5_seat_origin_cutter_mid、n_l5_fg15（E：坐面点在料里 → unknown） |
| **匹配键带 instance**：`declared_head_sides` 键 = (feature_id, instance, map_index, hole_index)；左右实例（mirror_y，同一份 STL、export_local 同坐标）同 outward 不是冲突、几何量一次并记录覆盖的 instance；"冲突"只在同一 instance 同一孔给了不同 outward 时成立；某个 instance 缺坐面 = 那颗螺丝未声明 → unknown | F-L5-2 第二半 | n_l5_seat_instance_key |
| **头侧孔全集 = `head_locator_map_indices`**：同一颗螺丝的 counterbore / spot_face 条目不是另一颗螺丝，不进"未声明"计数，也不作为叠厚/坐面的测量孔；locators 缺/重复/越界 → `head_side` unknown | 主线程 09-13（F02/F03/F10 "6/12 未声明"真因） | n_l5_seat_instance_key、n_l5_seat_origin_cutter_mid |
| **头判据只看头高区**：从坐面点沿 outward 退 60 mm 向内打，每条射线只认坐面外侧 ≤ 头高（`frozen.yaml:P_dict.m<N>_head.h`）内的第一张进料面；更外侧的杯壁/另一堵墙不算坐面；头高区内无进料面 = 头下悬空；足印环带在坐面头侧 1e-3 mm 处全是料 = 坐面点在料里 → unknown（不报成悬空） | F-L5-2 第二半 | n_l5_fg15（C 远端台阶绿、E 料里 unknown）、n_l5_head_on_pilot_hole（悬空红） |
| **阈值 src 进判据**：criterion 写"阈值 src=measured/datasheet/assumed/缺"；无 src → unknown；assumed → 照判 + `threshold_assumed` WARN。`engagement_by_joint_type.*.range_mm`、`screw_length_rule.tip_margin_mm`、`hole_diameter_mm`（L5 只当关联容差）本轮**未**纳入 | F-L5-4 | n_l5_threshold_src |
| **结构化字段优先于自由文本，两者不一致 → unknown**：自攻盲深先读 `fasteners.yaml:<组>.pilot.pilot_depth_mm`，再退回 joins/location 正则；都有且不一致 → `screw_length_rule` unknown 并把两个数写进 detail | F-L5-5 | n_l5_pilot_depth_priority |
| **目标区间只认 frozen.yaml 声明**：碰撞在 `frozen.yaml:joint_axes[].target_range_deg` 内 → BLOCK（件的问题），全部在区间外（仍在上游扫描域内）→ WARN（工作空间问题：控制端限位，不是件的问题）；两两组合按两轴都在各自目标区间内算"区间内"。缺失 / lo≥hi / 非数 / 超出扫描域 / 人为收窄（≠上游值）却没有 `date`+`reason` → 该关节相关判据 unknown，不许退回"全部 BLOCK"或"全部放行" | F-L6-3 | n_l6_target_range_severity、n_l6_target_range_declared |
| **BAM 摩擦三处同步**（09-17）：发布 `m1.json` / 包内 `s288_m1.json` / 两个生成 MJCF 的 `chosen_actuator` 的 friction_base·friction_viscous·armature·max_torque 相对差 ≤0.2%；缺文件或缺 class → unknown。台架重测（bench BN18）改一处没改另一处 = 训练里两套摩擦 | 用户 09-17 | n_l6_bam_params_sync |
| **生成物要对着当轮实测重判，不许因"需要收窄"本身判红**：`collision_free_range` = 生成的 `sim/duck_s288/robot_walk_s288.xml` 每条 `<joint range>` ⊆ 本轮扫出的无碰撞区间（否则 BLOCK）；`gen_range_covers_target` = ⊇ 目标区间（否则 WARN，训练域被收窄）。那份 XML 是独立产物会过期，这不是同义反复 | F-L6-3 | n_l6_gen_range_vs_free |
| **网格不许比要抓的口袋粗**：两两组合统一 5° 网格（零度对齐、强制含端点），只在峰值格附近加密到 2.5°；criterion 写明步长，detail 记步长与加密姿态数。不为省时间放大网格（实测组合扫描 44 s → ≈350 s） | F-L6-1 | n_l6_combo_pocket |
| **豁免只认声明，不认"贴得近"**：最小距离判据只豁免 `relations.yaml` 里声明为 `contact` 的 `parties` 件对（`placed` 认单只实体、`part` 认该件号全部实例）；零位间隙 < 阈值但没声明的件对另发 `_zero/undeclared_zero_gap_pair` FAIL(WARN) 列出，不阻断、不豁免 | F-L6-2 | n_l6_undeclared_zero_gap |
| **算了就要判，home 不是旁注**：判据算出的每个量都得有一条 PASS/FAIL 接着；home 姿态扭矩单独一条 `static_torque_home`（同阈值），不许只进 detail | F-L7-3 | n_l7_static_torque_home_and_range |
| **评估点数 == 2 不算扫**：区间必须 lo < hi 且 `check_angles` 给出 ≥ 3 个点（2.5° 网格 + 两端），否则该关节 unknown；criterion 写明最少点数来自 check_angles 的构造 | F-L7-3 / N4 | n_l7_static_torque_home_and_range |
| **清单不全 → unknown，不给下界当绿**：整机汇总判据（static_torque / static_torque_home / com_in_support_* / head_mass_budget）在质量清单有缺口（qty>0 无质量、无 claim、认领数≠qty、定不出 body、有质量认领不到实体、qty 非整数、打印件未归属）时一律 unknown，detail 列缺口清单并只给"仅含已知质量的下界"文本 | F-L7-4 | n_l7_mass_gap_unknown |
| **阈值只有一个来源**：同一个门槛在 data 里出现两次时层只读 yaml 自述的权威键，废弃键仍在且不等 → FAIL(BLOCK, measured=差值)，相等 → INFO 提示删；不再退回 docs/ 散文 grep | F-L7-5 | n_l7_torque_source |
| **认领规则跟着元件走，不按 category 分支**：元件在 placed/ 里怎么认由 `components.yaml:components[].claim` 声明；没有 claim → `component_body` unknown（不静默漏掉）；认领到的实例数 ≠ qty → `component_qty` FAIL(BLOCK, measured=实例数) | F-L7-6 | n_l7_component_claim |
| **对数/常数/落地件/容差都从数据取**：镜像对数由 frozen.yaml 的 left_/right_ 声明推（criterion 写"== 数据声明的对数"）；home_rad 缺失 = unknown 不当 0；落地件只认 `parts.yaml:ground_contact`；筋 ±容差进 tolerances.yaml 带 src；src=assumed 的密度 / 稳定裕度 / 筋容差各发 `threshold_assumed:<键>` FAIL(WARN, measured=阈值)，src 写进 criterion | §2.6（L7） | n_l7_generalized |
| **声明式弹性接触（L4，hr41g 09-24）**：刚体扫掠超阈的**每一个**采样都落在 `fasteners.yaml` 卡扣组（`joint_type: pla_snap`）两侧特征盒之交（外扩 0.01）里、且至少一侧特征写了 `features.yaml:<特征>.elastic_contact {material: PETG（须与 parts.yaml 一致）, interference_mm: {v>0, src}}`、区内体积 ≤ 过盈 × 面积（`contact_area_mm2` 或核心盒最大面）、路径末端仍证明脱离 → `motion:*` / `elastic_contact_declared` 降为 **WARN，永不 PASS**；区外 / 超上限 / 声明缺项 / 末端未脱离 → BLOCK 照旧；拆序里靠弹性拆出的件之后每条挂 `disassembly_prereq_elastic` WARN | hr41g §2 | n_l4_elastic_declared_warn / _outside_zone / _over_cap / _missing_decl |
| **扫掠型禁入体（L3，hr41g 09-24）**：`keepouts.yaml:<KO>.bounds_world_mm` + `sweep_direction_world` / `sweep_len_mm` / `moving_components` → 包络盒沿方向平移的凸包（精确 Minkowski 和）对 placed/ **全部实体（打印件 + 元件）− 随行件** 求交；`exempt[segment=all]` 照算记数不计峰值；另判 `keepout_sweep_clear_at_end`；方向/行程/随行件任一缺 → `keepout_sweep_declared` unknown，不代跑（首例 KO22，09-25 落盘） | hr41g §3 | n_l3_ko22_sweep |
| **禁撑区按特征 id 纳入（L1，hr41g 09-24）**：`printability.yaml:no_support_zones.feature_zones.<特征 id> = {tier: forbid\|removable, face_margin_mm: {v>0, src}, why, src[, post_process]}` 把 kind 表以外的实体特征（卡扣/舌片/倒钩）纳入；区 = 每实例 `box_export_local`/`bbox_mm`/`prism` 外包盒各向外扩 margin（实体盒不外扩 = 恒 0 点假绿）；条目坏 / 特征不存在 / 与 kind 双重纳入 / 定位不到 → unknown；只有本件确用到时才进 zones_sha256（首例 B03-F03..F06 removable，09-25 落盘；登记后须重切）。反例夹具 `_harness.base_data` 只留本反例特征清单里有的条目 | hr41g §4 | n_l1_feature_zone_snap |
| **沉头锥面坐面（L4 tool_access，09-25）**：`fasteners.yaml:<组>.tool_access[].seats[].cone = {included_angle_deg: {v,src}, top_d_mm: {v,src}[, head_d_mm]}` 时，坐面点 = 孔轴上锥口顶面，足印环带（同平面坐面的 3 圈 × 24 向）落在**锥面**上、沿锥面法向 ±0.001 判"料侧有料 / 空腔侧为空"（`tool_access.cone_seat_samples`）；不写 cone = 原平面判法逐字节不变。同一实体同一孔轴上的两个孔，只有特征声明了 `depth_mm` 且轴向相距 > 孔深才算两颗（F34 两毂同轴），否则仍按"同一孔凑两颗"unknown。L5 头判据仍按平面口径，锥面坐面在 L5 `head_seat` 为 unknown（待办） | hr41 §8 / 09-25 落盘 | n_l4_tool_cone_seat |
| **六角螺母穴（L2，09-25）**：kind=cavity、shape=prism_array 且写了 `geom.hex_prism {sides: 6, across_flats_mm: {v,src}, flat_normals_export_local: [逐位置平边外法线]}` 的穴按**对边 + 对角**量：每站 12 向（6 平面法向期望 r = 对边/2、6 角向期望 r = 对边/(2cos30°)），逐向直径当量容差同圆孔（hole_diameter_mm）；通盲 / 盲深同圆孔盲穴（`step_probe` 内圈 = 内切圆 −0.3、外圈 = 外接圆 +0.3）；`across_flats_mm` ≠ `nominal_d_mm`、法向条数不符或不 ⊥ 轴 → unknown，不回退圆孔探针；没写 hex_prism 的一律走原圆孔探针（逐字节不变）（首例 J02-F04 / J03-F03） | 09-25 落盘 | n_l2_hex_nut_cavity |

---

## 4. 产出

- `scorecard.json` + `scorecard.md`：17 件 × 8 层矩阵，每格 {状态, 数值, 证据数, 输入 hash, 过期?, 豁免?}。只有**全量跑**写它和 `scorecard.prev.json`；`--layers/--parts` 子集跑写 `scorecard.partial.{json,md}`（`partial: true`，不是放行依据）
- `negatives.cache.json`：反例包结果缓存（key 见元规则 8）；`gate.py --no-neg-cache` 强制重跑
- `gate.py --verify [scorecard.json]`：核对记分卡绑定输入是否还原样（元规则 6）
- `impact.md`：**这次改动让哪些格子变了**（换件后只读这个，不读全表）
- `fastener_bom.md`：从 features + fasteners 自动生成的螺丝采购单
- `mass_budget.md`：每 body 质量重心 vs MJCF、各关节静扭矩占额定比、头部质量上限（由颈部扭矩倒推）

## 5. 换件的完成定义（DoD）

改 `components.yaml`（和必要的 features/harness/keepouts）→ 跑 Gate → 读 `impact.md` → 清 BLOCK → 质量漂移超阈值就更新 MJCF `<inertial>` 并标注是否重训 → 同步 `06_CAD源码/` → 记分卡与 STL 同一 commit。

## 6. 改 Gate 代码的硬规矩（hr42 / L1 事故后，2026-09-25）

- **`layers/l1_printable.py` 任何改动（包括只为提速的改动）都必须连带重跑 `./.venv/bin/python -B tools/gate/calibrate_l1.py`**：标定记录 `l1_calibration.yaml` 的指纹包含该文件源码，不重跑则 `L1/_calibration` 判 BLOCK。约 2 min / 700 MB。
- 打印 / 交付 / 复审前的全链必须无缓存实跑：`DUCK_BUILD_CACHE=0 DUCK_CHECK_CACHE=0`（或各脚本的 `--no-build-cache` / `--no-check-cache`），并带 `PYTHONFAULTHANDLER=1`；日常改件用默认增量（命中格 evidence 标 `cached_from`），增量结果不是放行依据。
- 跑重活一律挂进程树 RSS 看门狗（机器 18 GB，超 10 GB 直接杀）；2026-09-25 L1 层曾因 H01 新朝向的支撑模型顶点爆涨吃到 63 GB + swap，根因与修法见 `docs/design_2026-09-17_bearing_rebuild/hr42_work/l1_incident/报告.md`。
- 检查原语缓存目录 `tools/gate/out/check_cache/` 会长到几百 MB，隔段时间清；并发跑多个 gate 子集时它是共用的，结果可疑先 `DUCK_CHECK_CACHE=0` 重跑再判断。
