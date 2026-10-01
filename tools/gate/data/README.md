# `tools/gate/data/` —— Gate 读的 12 个数据文件

规则只写不变量，一切"应该是什么"都在这里。**换件只改这里，不改检查器。**
全部由 `docs/gate/inventory_v2.json` + `inventory_original.json` + `comparison_original_vs_v2.md` 转换而来，不是重新盘点。

| 文件 | 一句话 |
|---|---|
| `components.yaml` | 19 个外购/占位元件：外形·质量·安装·接口·线缆·禁入·装入方向。换件的**入口**。 |
| `parts.yaml` | 17 个打印件（件号 L01…H03 为 key）：build 函数名·材料·打印朝向·镜像·装了哪些元件。 |
| `features.yaml` | 151 条特征：孔/沉头/槽/凸台/夹持面/让位/扫掠刀。只定位与追溯，判据去 `tolerances.yaml` 取。 |
| `fasteners.yaml` | 30 组 157 颗螺丝（+2 扎带 +2 卡珠）：规格·叠厚·咬入·**每颗自己的 `tool_envelope`**·驱动方向·状态。 |
| `assembly.yaml` | 19 步装配序（含 `already_installed` 累加）+ 8 条拆卸序。第 4 层照它逐步扫掠。 |
| `relations.yaml` | 22 条功能关系 + comparison C1/C2/C3 三条 Gate 条目（IMU 外参 / 少掉的轴承 / L03 刚度）。 |
| `keepouts.yaml` | 21 个禁入体：视锥·声孔·摆动扇区·电池抽出·接口·刀路·线束空间。 |
| `harness.yaml` | 9 束线：折线·径·最小弯折半径·两端插头·长度预算·跨哪个关节。现状 5 束完全没模型。 |
| `frozen.yaml` | 不许动的：15 条关节轴线·电池仓 26 项·SHELL_BOSS·H01 立柱·6 个原版 STL 的 sha256·躯干 IMU 位姿·C5 只对照不打印名单·C6 接口预算。**唯一允许硬编码世界坐标的文件。** |
| `tolerances.yaml` | 公差表：咬入按连接类型分，配合按 **材料 × 打印方向 × 特征类型** 分桶；顶部 `placeholder_summary` 报"还有几个配合靠占位值"。 |
| `waivers.yaml` | 豁免。**现在是空的。** 只有 WARN 可豁免，绑定输入 hash，输入一变自动失效。 |
| `bench.yaml` | 14 项实机验证：只放离线算不出来的，判据数字现在就写死（`value_status: proposed`），到货逐项打勾。 |

## 换件时改哪几个文件（对应 `tools/gate/README.md` 第 5 节 DoD）

1. **必改** `components.yaml`：那条元件记录（外形/质量/孔阵/接口/线缆/装入方向）。
2. **跟着改**（按新件是否动到）：
   - 动到孔位或让位 → `features.yaml`（该件的特征）+ `parts.yaml`（`houses`）
   - 动到螺丝 → `fasteners.yaml`（长度、叠厚、`tool_envelope`）
   - 动到接口/视锥/声孔/抽出路径 → `keepouts.yaml`
   - 动到线 → `harness.yaml`
   - 动到装/拆顺序或方向 → `assembly.yaml`
   - 动到配合（新的间隙/过盈/滑配）→ `tolerances.yaml` 的对应桶 + `relations.yaml` 的那条关系
3. **一般不改**：`frozen.yaml`（改它 = 解冻流程，要人签字并说明是否重训）、`bench.yaml`（除非新件带来新的实机风险）、`waivers.yaml`（只增不改）。
4. 然后：跑 Gate → 读 `impact.md` → 清 BLOCK → 质量漂移超阈值就更新 MJCF `<inertial>` → 同步 `06_CAD源码/` → 记分卡与 STL 同一 commit。

## 字段约定

2026-09-12 清红补充（只声明事实与检查义务，不是放行标签）：
- `parts[].placed_instances`：按 BOM `qty` 列出全部导出实例，首项为左件；缺失、重复、跨件复用均失败。`motion_instances` 给每个 placed STL 唯一的 MJCF body 归属；目前 52 个。
- `features[].geom.instances` 必须有 `count_kind: instances|cylinders`，分别表示组数或展开后的总柱数；组内阵列的 `count`/`holes` 是孔数，必须等于坐标数。重复整组不能补齐漏孔。同心异径或分段柱仍是不同几何。
- `assembly_order[].required_motion_groups` 独立声明动作 ID、工位、移动组及在场全集；`motions` 必须逐一覆盖。`sense: withdrawal_from_assembled` 表示从最终配合位抽出，反演为装入；全体刚性组沿同一向量移动，终点须完全脱离。
- `fasteners[].head_locator_map_indices` 先声明头侧孔全集，`tool_access` 必须覆盖它。每个 `state_ref` 指向 `motion:<step>:<id>` 的工位或 `tool:<id>` 的显式拧紧工位；坐面通过 `map_index/hole_index` 绑定已有 `feature_hole_map`，声明 `point_export_local/outward_export_local/instance`，不能拿孔中点代替。同一实体同一孔轴不能跨映射重复凑数。
- `features[].geom.hole_positions_mm` / `geom.instances[].hole_positions_mm`（= `fasteners[].feature_hole_map[].holes`）是**刀心**（切刀中点），不是坐面、也不一定在件上（40 mm 的刀切 3.6 mm 的板，刀心离件 15–19 mm 甚至在件外）。L2 用 `axial_span_mm` 兜住孔身范围；其它层不许拿它当坐面或射线起点，只能当轴线锚（校验点在孔轴上）；头/坐面一律用 `fasteners.yaml:tool_access[].seats[]`（`point_export_local` / `outward_export_local`，键带 `instance`）。（2026-09-13，F-L5-2 第二半）
- `tool_envelope.bit_d_mm` 是本组批杆包络；`available_shaft_len_mm` 是可用杆长。旧 `shaft_len_mm` 多数是反算需求下界，`channel_*` 是内腔，两者都不能当可用工具规格。缺实际数据继续失败。
- H03-F02 是孔身，H03-F04 是四段扩口。F22 的径向自攻必要条件为 −0.1 mm，属于配合不兼容；不能归为“补字段即可放行”。（2026-09-13 起已过时：09-12 I4 在 H03 先填 Ø3.6 后自钻 Ø1.7×7.0，H03-F02 重写为自攻底孔、H03-F04 删除、新增 H03-F05 填柱，径向重叠 +0.15；见 CHANGELOG 2026-09-13『Gate 数据对表 build #7』。）

- **`provenance`**：每条记录必须有，值是列表。格式：`duck.py:行` / `s288.py:行` / `ankle_split.py:行` / `asmcheck.py:行` /
  `mechanical_audit.json:键` / `inventory_v2.json:路径` / `inventory_original.json:路径` / `MJCF:行` / `comparison_original_vs_v2.md:编号`。
  **没有出处的条目不许写进来。**
- **数值字段**：统一 `{v: <值>, src: measured|datasheet|assumed}`，可选 `src_note` 记原始表述。
  - `measured` = 实测（`s288.py` 里标 `[量]` 的、射线量出来的、从原版网格量出来的、audit/report 里的数）
  - `datasheet` = 手册 / 商品页 / 标准件规格
  - `assumed` = 采用值（`inventory_v2.json:assumed_values` 那 20 条全部在 `tolerances.yaml:assumed_values` 里，全是 `assumed`）
  - **混合表述按最弱一环取 `assumed`**（元规则 4：未知=失败，保守优先）
- **搬不过来的值**：`v: null` + `unknown_class` + `unknown_reason`。`unknown_class` 只用这 6 个：
  `not_measured`（没量）/ `not_audited`（几何在但没跑检查）/ `not_modeled`（CAD 里就没有）/
  `not_selected`（选型未定）/ `not_applicable`（本来就没有这个概念）/ `no_source_given`（清单没给来源）。
- **`deliberately_absent` / `deliberately_added`**：comparison 文档 A 段 16 条 / B 段 11 条的落地记录，带 `comparison` 编号和理由。
  **看到"原版有我们没有"就想补回来之前，先读这两段。**
- **`discrepancies` / `conflicts`**：两份 inventory 互相对不上的地方（D-COMP-01…03 / D-PART-01…02 / D-FAST-01…03 / D-FROZEN-01…03 / seat_d / pilot_hole_d）。别当噪声删掉。
- 不许在规则数据里硬编码"这个件在世界坐标哪里"，除非它本身是冻结项。特征用相对本件基准的描述；判据用范围不用单值。

## 2026-09-13 审计第二批新增字段（换件只改这些，层不再写死）
- `components.yaml:components[].claim`：元件在 placed/ 里怎么认领（`by: mjcf_servo_geom | envelope_ring | envelope_box | placed_prefix`，见文件头注释；L3/L7 同一份）。`envelope_solid`：第 3 层独立建实体的声明（`shape: ring{bore_mm, od_mm, t_mm} | box{size_mm, overall_mm, exact} | none{why}`（box 的三边按 overall_mm 与 placed OBB 三轴长度配对；两个 overall 值差 <0.5 且对应 size 不同 → unknown）），落位只借 placed 的 OBB，尺寸不许抄 CAD；没声明 → `envelope_solid_declared` unknown。
- `keepouts.yaml`：`checked_in_layer`（4/6）+ `covered_by_subjects`（该层格子名）—— 扫掠/运动类禁入体的义务派给真查它的层；KO01 `dims`（plug_top_x/wire_out/conn_z/plug_clr/plug_y_in/side_exit_y/window_x/hub_keep_d）+ `availability_table.rows[].mode_pos_y|mode_neg_y`（独立实体，不借 CAD 切刀；09-20 背插模型）；KO06 `servo_frame_of_joint`、KO07 `axis_joint`、KO11 `probe_zone_z_mm`、KO12 `exempt[]{part, segment, why}`、KO15 `fov_deg / apex_world_mm / optical_axis_world`。缺任一 → 对应判据 unknown。
- `frozen.yaml`：`joint_axes[].target_range_deg{v, src[, date, reason]}`（第 6 层目标区间：缺省 = 上游 MJCF；人为收窄必须带 date+reason）；`battery_bay_26.measured_on{bay, door}`、`h01_sbc_posts.measured_on / post_axis_world`、`imu_pose.measured_on / mount_hole_d_mm`（第 3 层冻结项实测的落点）。
- `printability.yaml:parts.<件>`：`down_world / fill_density_percent / perimeter_generator / slicer_args`（切片脚本逐件配置，缺 → 退出）；`no_support_zones.tiers / per_feature_overrides`（两级禁撑区）。
- `tolerances.yaml`：阈值节点必须带 `src`（`assumed` 照判 + `threshold_assumed` WARN，缺 → unknown）；新增 `feature_check_tolerances.frozen_dimension_mm / envelope_vs_placed_mm`、`load.min_section_and_rib.rib_declared_vs_measured_tol_mm`。
- `parts.yaml:parts[].ground_contact: true`：第 7 层落地件（不再按材料文本猜）。
