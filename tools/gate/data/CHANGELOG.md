## 2026-09-25 hr43e —— hr43d 全链暴露的 6 项修正 + 协调员 20:10 收尾四件；报告 docs/design_2026-09-17_bearing_rebuild/hr43_work/hr43e_修正报告.md
几何（build_fast #1 --only H03,H08,H09；#2 --only H03,H09）：静态 0 / mechanical PASS（未闭环仅 B01）/ 扫掠 243 → 238（head_pitch 少 −90° amp_bracket × shell_L）；旧检查域内 0。
- **L6 目标区间内清零**：H09 竖板 y 29.5 → 32.4 起、脚板 y 32.4..37.2 / x −24.8 起、F38 孔与 H03 凸台 y 33 → 34.7（头俯仰 × 颈俯仰只绕 y 转，shell_L 头系 y ≤ 31.76）→ H09 区间内 33 → 0；H08 +y 下纵梁 +x 端 81.8 → 75.5、托面前下角 x ≥ 76 / |y| ≥ 12.5 挖到 262.8（新 H08-F07）→ H08 区间内 3 → 0；两格 BLOCK → WARN。H09 刻字改竖排（stamps.py）。
- **收尾（协调员 20:10）**：① H03 凸台顶 224.5 → 225.0、H09 脚板 / 竖板底抬 0.5、F38 底孔 4.3 → 4.8（尖余 0.8）；② F36 M2×5 → M2×6（用户有 M2 自攻 PA ×6）咬 3.4 → 4.4，H09 两立柱背面加 Ø4 × 1.0 背凸台（新 H09-F05），Ø1.7 通孔 5.0；③ H08 扎带头放托板 +y 侧上方 / 竖板一侧、不放前下角（只登记）。
数据：fasteners（F36 / F38 joins 加引号 —— L5 整层崩溃根因；起子 PH1 杆长 75 定义一次（锚 &avail_shaft_ph1_75）、PH0 38 组别名引用 → tool_reach_declared 20/20 PASS；F36 / F38 规格 / 底孔 / 坐面）、components（tape_foam_1mm 撤成 bus_adapter note、counts 28 → 27；amp / mic / sbc envelope_solid → none + 多盒记录 —— 单盒按 OBB 中心落位压进件是登记口径，真元件 × 件 = 0）、features（+H08-F07 +H09-F05，329）、parts、assembly（步 18 文字）；negatives/_l7_fixture 删 tape_foam_1mm（n_l7_* 7/7）。assembly.yaml:3160 / parts.yaml:1183 两处同类未加引号流式项一并加引号。
**没跑全链**；L6 real_pose_collisions 过期待重采。

## 2026-09-25 hr43d 全链实跑（chain_d，18:02–18:35，无缓存）；汇总 docs/design_2026-09-17_bearing_rebuild/hr43_work/chain_d/汇总.md
Gate BLOCKED ✅147 ❌426 ⬜6 🪦1，反例 82/82；真实姿态 119/5601（= hr43c，新增 ankle_foot_R / ankle_rear_arm_R × zz_adapter 两对，alpha_stand）；build 扫掠 243。**L5 功能层整层崩溃**（fasteners F36 joins 首项未加引号被 YAML 拆成 4 项，第 2 项 float → l5_function TypeError）→ ❌ 476 → 426 大头是这一层缺席、不是转绿；hr43e 修。其余层正常；新件 H08 / H09 L6 目标区间内 3 / 33 条（BLOCK）、L3 功放 / 麦克风 / Radxa 包络压进、L7 tape_foam_1mm claim none、L4 F38 缺 available_shaft_len_mm —— 均交 hr43e。

## 2026-09-25 hr43d —— 转接板平放螺丝托板 H08（B′）+ 功放挪后脑 H09（E′）+ 删无用件 + 电子件加固；报告 docs/design_2026-09-17_bearing_rebuild/hr43_work/hr43d_改件报告.md
几何（build_fast --only H01,H03,H05,H08,H09 #1；#2/#3 --only H08,H09 重生成 zz_*）：静态 0 / mechanical PASS（未闭环仅 B01）/ 扫掠 243 = 216 + 27（全是 head_pitch −65..−90° 的 zz_amp / H09 × 躯干，同姿态 H03/H05 已交躯干；旧检查域内 0）。
- **新件**：H08 转接板托板（竖板照 H07 贴 H01 三根旧立柱 F35 复用 + 托面 2.0 厚 z 262..264 + 0.5 胶坑 36×26 + 2 扎带位 + 两条下纵梁；板脚印 x 51.8..81.8 = 研究位 +x 5.8 让 F22#3 起子通道 KO09）；H09 功放支架（竖板 + 两 Ø4 立柱 F36 + 脚板 F38 2×M2×6 拧 H03 新凸台 F24/F25）。H07 退役。
- **electronics**：功放 AMP_S_C/R → 后脑 +y (−10.8, 28.8, 235.8)、板竖放元件面 +x（孔改局部系开）；转接板平放占位 ADP_FLAT / adapter_boxes；排针直插杜邦功放 5 腿出壳即弯改朝 −x（40 号斜 45°，逐针天花板余量 ≥1.39）。
- **head / head_top**：H03 删 F14 槽 / F22 平床 / F23 胶坑 / F15 USB 插头井 / USB 壁切口，加 F24/F25；麦克风台 / UBEC 托条下沉 0.8 放 1 mm 泡棉；H05 删 F06 压筋 / F09 叉齿，喇叭肩面抬 0.5 放泡棉环。
- **wiring_head**：ADP 平放口位（L 边 −x，A 边 +y：PH 57.8 / XT30ext 71.7；B 边 −y：Type-C 59.9 / XT30in 74.4）；19 根线重走（功放 5、喇叭、USB、12 V、UBEC 进 + 出 2、S1 两腿、麦克风 6；S1 转接块与嘴 PH 短线留 hr43c 原位）；CSI 板上方段抬 0.35、折 1/2 挪 x 64.5。check43d_final：线∩件（非预期）≤0.087、三关节 + 嘴扫 0；弯折低于声明 12 根（端头拐角，声明值 assumed）。
- **注册**（简报文件清单外，新件必需）：build_fast.py / build.py 加 H08 / H09、去 H07；stamps.py 加 H08 / H09 件号。
数据（定点文本改，旧值 *_was_until_2026_09_25_hr43d；注释行数不变）：components（bus_adapter fixation zip_tie by [H08]、cable_dupont_ff captive by [H05]、tape_foam_1mm claim none + mass 0.3 assumed —— 修 hr43c 全链三条登记红；amp / speaker / mic / UBEC / XT30 加固登记）、features（−14 +14）、parts（36）、fasteners（F35/F36 改接 H08/H09，新 F38）、assembly（步 18 P″、各组在场件）、harness（HB01/03/04/06/08/10/11 hr43d_route）、printability（H08/H09）、keepouts KO14、frozen 注。
**没跑全链**；快车道 hr43d_head 见报告。

## 2026-09-25 hr43c —— 后脑零鼓包 + 排针直插杜邦四件 + 复审 #2 B1/B2 + M4 登记；报告 docs/design_2026-09-17_bearing_rebuild/hr43_work/hr43c_改件报告.md
几何（build_fast --only H01,H03,H05,H07 #1/#2：静态 0 / mechanical PASS（未闭环仅 B01）/ 扫掠 216 = 215 + head_pitch −90° zz_sbc × zz_xt30 0.315（目标 −50° 之外；dy +1.0 把 USB-C2 口壳推进平衡头 y 带 0.17））：
- **electronics**：板 SBC_A_C (−3,−2,257) → (−3.5,−1.0,256.5)（协调员四档 dy 比选，逐针最差 @Rc2.0 +1.127/+1.129/+1.041/+0.821 → +1.0）；孔 / 顶面包络随板；板角 R3 → 2.45（官方 DXF）；zz_adapter40 退役 → 新占位 zz_gpio_dupont（R2 针位 14 个杜邦壳 2.54×2.8×14 十字 + 17 段出壳即弯管，UBEC 4/34 用 OD1.6、Rc 2.0/4.0）；功放 AMP_S_C y 0 → −2.0（B1）。
- **head_top（H05）**：F10 鼓包退役（外形 = 放大原版）；新 F15 = 排针上方横筋局部削 + ±y 两端内皮减薄 ≤0.55（壁 ≥1.227）；吊柱随板（柱长 8.0/16.9/9.86/18.17）、(8,±) 贴杜邦壳削平离 1.0；F14 板角让位删（板↔原壳 1.284）。
- **head**：H07 托面 / 立柱 / 纵梁随功放 −y 2.0 + F22 右前脚 Ø5.3 缺口过孔 + web（F22 4/4 起子路径通）；H01 删空着的 (−29,239.5) 立柱；H03 转接板 +x 壁 + 立柱 → 竖粘平床 F22 + 0.5 胶坑 F23（协调员 14:55，15:20 起位置待用户重定）。
- **wiring_head**：dupont OD 1.3 → 1.4；PINMAP R2（6→14、39→20、1→17 一分二）；20 根线 Radxa 端改壳顶接续点重走（_HR43C_ROUTES）；CSI 候选路径 csi43c_a..d（3 折，对件 / 线 0 交，HB06 → 候选）；USB 弯头随板。
数据（全部定点文本改，旧值留 *_was_until_2026_09_25_hr43c；注释行数都不变）：features（−H05-F10/F14，+H05-F15、H07-F06（half_only）、H03-F22/F23；改 H05-F11/12/13、H07-F02/03/04、H03-F14、H01-F03/04；322 → 324）、parts（H01/H03/H05/H07 体积 / feature_count、motion_instances zz_gpio_dupont）、components（gpio_rightangle_40p qty 0；cable_dupont_ff 认领 zz_gpio_dupont + 实量 + 一分二 3；+tape_foam_1mm；27 → 28）、harness（HB04/05/08/10/11 hr43c 线、HB06 候选、HB01 usb）、assembly（head_shells_on 按 P′：−zz_sbc +amp_tray/zz_amp；zz_adapter40 → zz_gpio_dupont；步 18 plugging_steps_hr43c）、fasteners（F35 按 Lane C 口径；F36/F37 随件 + schema 缺口记名）、frozen（H01 立柱删一根例外）、keepouts（KO14 注）、printability（H03/H05/H07 注；快车道 merge 刷了切片记录）、manifest（zz_gpio_dupont）、head_specs（head.py 指纹）。
**没跑全链**；快车道 hr43c_head L1/L2 与 hr43b 同红（H07 首次进快车道，L2 0 BLOCK）。

## 2026-09-25 hr43 —— Radxa 搬后脑（方案 A，用户 09-24 17:25 定）+ 直排针功放托板 H07 + 顶壳鼓包/吊柱 + H03 6700 下半环恢复；报告 docs/design_2026-09-17_bearing_rebuild/hr43_改件报告.md
几何（build_fast --only H01,H03,H05,H07 #1–#4；#4 终版：静态 0 / mechanical PASS（未闭环仅 B01 基线）/ 扫掠 215 = 基线 210 + 6 − 1，增减全在 head_pitch −87.5..−90°（目标 −50° 之外、旧检查域内 0）/ H07 单实体）：
- **electronics**：sbc_boxes → 方案 A 平放 (−3,−2,257)，板 z 256.2..257.8、四孔 (−14.5/8.5, −31/27)、DXF 禁布 r3；新占位 zz_adapter40（盒 A x −2..26 × y −28..24 顶 279.8，板边外那截 z ≥ 266.5；盒 B x 26..44 × z 266.5..274.5，assumed，N03 横滚扫掠 + 喇叭后挡夹出来的带）；amp_boxes → 直排针版 (56,0,263)。旧函数 *_was_until_2026_09_25_hr43。
- **head_top（H05）**：F10 后脑鼓包（盒 A 上内面 ≥ 280.85，最高抬 8.26 @ (−2.5,−29)，壁 2.0，内面后坡/外面前坡 ≤45°）；F11 四吊柱 Ø6 泪滴（柱端 Ø5.4、长 8.58/20.06/10.48/20.03）+ 长柱筋（+x 片 45° 下沿延到内面）；F12 Ø1.7 底孔；F13 SoC 出风槽 5×1.6；F14 Radxa 板角让位（壳↔板 0.734 → 1.04）；功放卡座 F04 拆除（旧 build 留痕）。
- **head**：新 H07 build_amp_tray（竖板贴 H01 旧四柱端面 x 66，3×M2×8；托面 + Ø4 立柱托功放 2×M2×5）；H03 刀序修复 → 6700 下半环恢复（截面 46/46）。build / build_fast / stamps 注册 H07。
- **wiring_head**：_HR43_ROUTES 20 根重走（15 杜邦 17 腿 + 喇叭 + 嘴扁线 + USB-C 15 cm）；线∩件最大 0.087（基线）/ 新增 0.020；CSI 200 未建（HB06 未闭合，待用户答）。
数据（全部定点文本改，旧值留 *_was_until_2026_09_25_hr43）：parts（H07 新件、counts 35；H01/H03/H05 role/houses/体积）、features（−H05-F04，+H05-F10..F14、H07-F01..F05、H07-S01；311 → 321；H03-F17 注）、fasteners（F25 停用 qty 0；+F35/F36/F37；screws 176 → 181）、components（sbc/amp fixation、camera_csi 200、+gpio_rightangle_40p；26 → 27）、harness（HB01/04/05/06/08/10/11）、assembly（步 18 装序 P′ + plugging_steps_hr43 + motion a/d + 3 个 tool_states）、printability（H07；H05 注）、frozen（h01_sbc_posts 例外 H01_posts_no_longer_carry_Radxa_hr43）。tools/sim/cad_geometry_manifest.json body_for_part + amp_tray / zz_adapter40（hr38_refresh_manifest --apply，build #4 后）；head_specs.json head.py 指纹刷新。
**没跑全链**（build / slice_l1 / audit_motion / policy_pose_collisions / gate 都没跑）；features counts 321 不含 Lane C 并发加的 E01-F01/F02、E03-F01（列表 324 条）。

## 2026-09-25 hr41 遗留落盘（主设计 Lane C）—— hr41g 待补 yaml + Gate 判据三处缺口；报告 docs/design_2026-09-17_bearing_rebuild/hr41_遗留落盘_2026-09-25.md
数据（全部定点文本改，旧值留 *_was_until_2026_09_25）：
- **features**：B03-F04 `elastic_contact`（PETG，过盈 0.25 assumed）；B03-F05 第二实例 `box_export_local` = nominal_box_mm；J02-F04 / J03-F03 `geom.hex_prism`（六角穴对边 4.2 + 平边外法线，nominal 不改）；新增 E01-F01（window Ø24.4）/ E01-F02（boss Ø32）/ E03-F01（boss Ø3.4）。311 → 314。
- **printability**：`no_support_zones.feature_zones` 登 B03-F03/F04/F05/F06（removable，face_margin 0.45 assumed，主设计定档）→ B03 zones_sha256 变，待全链 slice_l1 重切。
- **assembly**：步 21 battery_lid 与拆序 seq0 末段加 −x 20。
- **keepouts**：KO22 `sweep_direction_world` [-1,0,0] / `sweep_len_mm` 60，status → checked；counts 补 checked_offline。
- **fasteners**：F34 声明 6 个锥面坐面（seats[].cone，工位 motion:18:f_J01_up），feature_hole_map 孔锚改 J01 export_local。F27 重复键 hr41 已清（yaml.compose 全库 0 组）。
判据（tools/gate）：tool_access 锥面坐面 + 同轴异位孔按孔深区分（n_l4_tool_cone_seat）；L2 六角穴对边/对角量法（n_l2_hex_nut_cavity）；L3 fixation 采样 seed=0；反例夹具 _harness.base_data 只留本反例特征的 feature_zones、n_l3_ko22_sweep c_real 取"去掉两项的真 KO22"。

## 2026-09-24 hr41 —— 头部阶段二（复审 #1 B1 + M1–M7 + 摄像头贴板 + MINOR；Radxa 搬后脑相关项转 hr42）
几何（build_fast --only，#1–#5，每次静态 0 / 张嘴 0..30° 0 / mechanical PASS / 扫掠 210 对同 hr39f 基线）：
- **J01 v2**：两臂带内内面 52.05、外侧加厚 1.0，毂补平 Ø20，两毂各 3 个 M2 沉头孔，r 7.1 @ 350/130/250°（不对称防错）。法兰 6 孔、沉窝、垫台、轴颈都取消。
- **新件**：J02 +y 法兰转接盘（3×M2×5 上法兰，装一次不拆，压 3 颗螺母）；J03 −y 轴颈盘（进 6700 内圈，压 3 颗螺母）；H06 摄像头压框。
- **H05**：两侧开 21 宽嘴毂槽；加 2 个保持凸台（x 64、y ±48.5）；F06 压筋改叉形，增加 −x 齿。
- **H03**：转接板 +x 立柱 ×2；保持螺丝过孔 + 锪平 Ø4.6 到 z 232.30；UBEC +y 窗改 U 槽，扎带改沿 x 绕。OTG 地板口做了又撤（Radxa 在 hr42 搬走）。
- **H04**：摄像头贴板（去立柱，Ø23 通窗，0.8 浅坑，四角定位销，压框凸台 ×2），麦克风槽改下开口，取消 E01 销孔。
- **眼睛**：E02 改 Ø24 圆筒；E03 凸 0.2；E01 去销。
- **zz_camera**：前移 5.9，PCB 补 4×Ø2 孔。
- **head_specs**：N08 耳按 120° / −60° 登记；起子障碍改为步 11b 工位。

数据：
- **frozen**：head_pitch 目标 [−50, 90]；h01_sbc_posts.exceptions 2 条（m6）。
- **tolerances**：新桶 machine_screw_hex_nut（null / unknown）。
- **features**：293 → 311。
- **parts**：34 件。
- **printability**：新增 J02 / J03 / H06。
- **fasteners**：
  - F27 改为 H06 2×M2×8，工位 h04_bench。
  - F28 改为 J02 3×M2×5，工位 j02_bench。
  - 新增 F33_H05_keep（2×M2×8，工位 h05_keep_screws）。
  - 新增 F34_jaw_hub_to_adapters（6×M2×5 沉头机牙 + 6 螺母；锥面坐面 L4 不支持，tool_access 不声明）。
  - F30 注。
  - counts 173 → 176 + 6 螺母；「全自攻」claim 加 correction。
- **assembly**：
  - 步 18 = 装序 P 7 个 motion + 插线步 M6。
  - 步 19 起补 J02 / J03 / H06。
  - head_shells_on 改为 c 步。
  - 新工位 j02_bench / h05_keep_screws。
  - 步 11 F30 注；拆序注；m8。
- **components**：
  - camera / mic / UBEC / 转接板的 fixation 按 hr41 重写，附六向自核 fix41.json。
  - sbc 写「固定方式待 hr42：H05 四柱」。
  - 6700ZZ by J03-F01。
  - mic insert_dir 补上。
- **harness**：
  - HB06 顶层登规格 150 / 11.3 / 16×5，status「走线待 hr42 按后脑位置重走」（m7）。
  - HB01 USB 改 15 cm（m12）。
  - HB03 xt30 4.83/6.0、HB04 ubec_in 2.62/4.0，登「下限 assumed、待实测」。

自核（非 Gate）：
- 装序 P a–g 带线 / 不带线全 0；张嘴 −5..30° 全 0。
- 反例：旧 H05 × J02 1.819、旧 J01 × H03 783。
- 坐面 F28 / F27 / F33 全过；线束 check41_final 与基线同。
- 头部审计 PASS 100 / NOT_RUN 3 / FAIL 1（N07 老问题）。

**Gate / 全量链 / audit_motion / 真实姿态都没跑**（协调员 16:05 禁令）。报告：docs/design_2026-09-17_bearing_rebuild/hr41_头部阶段二.md。

## 2026-09-23 hr39c 4c —— 按 4a 终版 build（21:51）补齐 4b 留下的 4a 遗留 TODO（yaml 28 处 + 报告 5 处）、刷新体积、重跑 feat_calc
几何不改。**features**：H04-F01 / F02 / F04、E01-F01、E02-F01 注明摄像头倒装绕光轴转、光轴不动、几何不变（todo → todo_resolved_2026-09-23_hr39c_4c）；H03-F14 加 `usb_wall_cuts_2026-09-23_hr39c`（−x 壁 y −13.6..−4.4 开口、y −24..−13.6 削到 z 239）；H03-F15 改 ⑧ 出线方向（+x → 缝里竖起）；H01-F03 加 `otg_relief_2026-09-23_hr39c`；H03-F13 加 `ubec_two_end_leads_2026-09-23_hr39c`（托条 + 墙底开口）；frames.per_part E01–E03 / H05 / J01 注改为 4a 复量相同，H01 / H03 / H04 的 export_stl_bbox_mm 换成 hr39c 实量（旧值留在 `_was_until_2026-09-23_hr39c`）。**parts**：H01 19.083 → 19.080、H03 40.631 → 40.622，其余 6 件复量相同；todo → changes_4a / todo_resolved；E01 / E02 notes；`jaw_soft_mass_2026-09-23_hr39c.refreshed_2026-09-23_hr39c_4c`（make_mjcf 口径 283.48 g，贴近实物仍 242.2 g）。**harness**：HB06 route / route_len 96.0 / 线长实测 142 / 触点奇偶 / open_items；HB01 usb_link 状态、净空、**purchase_spec（假设，下单核对）**。**components**：TF 卡对 ⑧ 的影响与换卡步骤、bus_adapter 注、camera_csi 加 `mount_2026-09-23_hr39c`（倒装）、buck_12v_5v 加 `placed_note_2026-09-23_hr39c_4a`（两头出线）。feat_calc 用 `hr39c_work/data4b/run_feat_calc_4c.py` 重跑（旧值读 `yaml_before/features.yaml`，避免平移两次；输出 `feat_records_4c.json`）：65/65，25 条记录 + 6 条平移与 yaml 逐项一致。电池 / 开关条目没动（另有评估）；counts 不变（features 279）。

## 2026-09-23 hr39c —— 整头改版数据登记（4b：头等比放大每侧 +10、新嘴轴、转接板挪到 Radxa 前面、圆眼三件；数据轮，几何 = build 第 3 次 19:02）
脚本 `docs/design_2026-09-17_bearing_rebuild/hr39c_work/data4b/feat_calc.py`（常量 → export_local，placed 快照逐条核 pos 有料/无料 65/65）+ `sync_4b.py`（文本级替换，原文件备份 `data4b/yaml_before/`）。**parts** 27 → 30（E01 白外圈 / E02 黑眼珠 / E03 高光）、头部五件 hr39c 块（体积 / 改了什么 / TODO）、J01 role 改新嘴轴、**motion_instances 补 bearing_jaw / servo__jaw_soft_jaw（hr38 漏登，L6 整层塌的根因）+ 圆眼三件、zz_ubec → jaw_soft**、新顶层 `jaw_soft_mass_2026-09-23_hr39c`（make_mjcf 口径 281.4 g / 贴近实物 ≈242 g）；**features** 264 → 279（删 H03-F08 tub；新增 16 条；重写 H01-F17 / H03-F09 / H03-F12 / H04-F03 / H05-F01 / J01-F01..F04；平移 6 条；frames.per_part 加 E01–E03）；**keepouts** KO01 row 15 嘴两口 free / usable 2、row 14 bus_uses both；**harness** HB01 头链 S1 → 14 → 8 → 7 → 6、S6 取消（一分二 5）、新买 PH 双头 10 cm、HB03/04/05/08/10/11 头内线长、HB06 CSI 定金色 22→15 同面线（线长待测）；**components** bearing_6700zz housed_by + H01/H03（hr38 L4 约 15 格 present 缺 bearing_jaw 的根因）、bus_adapter 槽/压筋/装入方向、S6 取消、PH 10 cm 采购、麦/UBEC/喇叭/功放新位；sbc_radxa_zero3w 记 TF 卡永久占位（−y 端，包络 67）与排针侧/背面分项实测（10.3 / 1.4 → 包络 13.3）；**assembly** 步 18 嘴子总成挪到脸板之后 + 新动作 eyes_onto_face_plate、zz_ubec 步 19 → 18、步 13 与三个早期工位去掉误塞的嘴三件、步 20 / 拆序 0 / head_on_headpitch 补全头件；另修 hr38 遗留口径：步 3/4 parts 的 '6700' → bearing_left_ankle / bearing_right_ankle（型号按外径解析会把 bearing_jaw 算进踝子总成 = hr38『约 15 格 present 缺 bearing_jaw』根因）、裸名 top_head_shell → H05 / head_top_shell（拆序 3 movers 补喇叭/功放）；离线用 L4 derive_present 核：全部动作 present ⊇ 推导最小集，缺 0；**fasteners** F30 耳 120°/−60°、F28 新嘴轴孔位；**printability** E01–E03 朝向。**TODO(hr39c-4a)**（4a 在改，主 agent 补）：摄像头朝向/位置（H04-F01/F02/F04、E01–E03）、CSI 线路由（HB06）、USB-C2 → 转接板线（HB01.usb_link）、OTG 让位（H01 立柱）、H03-F14/F15；件体积待 4a 重新 build 后刷新。**未动**：tool:head_shells_on（F22 从上拧要求顶壳不在，与步 18 顺序矛盾，hr37 起就红）、MJCF（全量链重生成，Radxa 重复计算那时修）。验证由全量链给出。

## 2026-09-22 hr37 —— H04 脸板 + 麦克风台 + 电子件占位实体（改件，全链；20:56 起，分数见记录 45）
几何：**H04 脸板**新件（替换原版 face_part：平板层原样保留、填旧孔、13.6 方镜孔、4 根 Ø4.5 立柱 Ø1.7×5 底孔、麦克风边浅槽、刻字内面）；**H03-F12 麦克风台**（2.0 台 + 三面墙 + 唇，Ø4.2 沉坑 + Ø1.5 声孔）；**T02** collar 面板足印以上壳皮削平（`tail.SWITCH_TOPFLAT`）。
placed 新实体：zz_sbc / zz_mic / zz_camera（jaw_soft）、zz_imu / zz_switch / zz_ubec（trunk_base），orig_face_part 不再导出。
数据（`hr37_sync_data.py`）：parts H04 + motion_instances + counts 26/253；features per_part.H04、H04-F01 window / F02 pilot_hole ×4 / F03 slot、H03-F12 boss（by_kind/by_part 同步）；fasteners **F27_camera_to_H04** 4×M2×4 自攻（tool_access 4 坐面 @PCB 背面 x 72.6，工位 h04_bench；杆长 unknown not_measured）；assembly 步 18 加 sbc_onto_posts / face_plate_into_groove 两组 + H03_bottom_shell 带 zz_mic、步 19 元件改 zz_*、步 20 / 拆序 0 / head_on_headpitch 随头 +4、tool_states h04_bench（26）；components 六件 claim + envelope_solid（UBEC = 腔包络 15×4.5×18，WAGO 无实体并注明原因）；printability parts.H04 down [1,0,0]；tools/sim/cad_geometry_manifest.json body_for_part；head_specs head.py 指纹；stamps H04。
待用户定：UBEC 选型（Matek 2 焊点 / 找 ≤15×4.5×18 带线款 / 头里再探）、WAGO 减到 0 改走转接板 XT30 拓展口。

## 2026-09-22 hr36 —— hr35 回归修：发布包 build_fast.py 同步 + F14 叠厚 3.55→3.40（数据轮，件同 hr33；15:21 起 采样 → Gate，分数见记录 44）
**结果 ✅134 ❌382**（历史最好）：恰好 `_release` 源码 21/21 与 F14 crosscheck 0/6 两条转绿，0 新红。文案待下轮：printability H03 note "hr35 抹平"→"接受不改件"、F14 engagement note 方向 +16.4 → +13.0。
fasteners **F14_headpitch_flange** `stack_mm` 3.55→3.40、`engagement_mm` 2.45→2.60：hr35 Gate L5 `stack_from_seat_crosscheck` 逐孔实量 [3.4]×6，记录值是 09-12 惰轮端面 −12.85 时的旧值（F14b 在 hr26 已改 3.40，F14 漏改）。发布包 `06_CAD源码/duckstructure/build_fast.py` 复制为仓库版（hr35 m-01 shell_R 一行修后未同步 → L0 `_release` `source_copy_sha256` 20/21 红）。

## 2026-09-22 hr35 —— F14/F14b 坐面 + 拆序 seq 0 + 注释/口径（数据轮，件同 hr33；14:50 起 采样 → Gate，分数见记录 43）
**结果 ✅133 ❌383**（hr34 ✅133 ❌382）：F14/F14b 起子路径三项 + disasm00 + L5 坐面四项转绿；新红 = tool_reach_declared ×2（实物杆长未知）、counterbore_depth/head_vs_neighbours WARN ×2、F14 stack crosscheck 差 0.15（旧值）、_release 源码 20/21（build_fast.py 未同步）→ hr36。
几何：**不改**。H03 梳齿（hr33 复审 M-01）四种修法都不干净（并集再 minkowski >40 min；grow 0.75 不消反出夹点；闭运算填孔切除体与 hr33 完全相同；口袋区开运算留碎片）→ 根因是壳皮在扫掠包络擦过处的薄鳍，不是刀参数；切片 rc 0 会丢掉，**接受不改件**（复审建议 ②，记录 43）。`head.py` 恢复 hr33 版。
数据：fasteners **F14_headpitch_flange / F14b_headpitch_idler** tool_access 坐面重生成（`hr35_seats_f14.py`：hr31 探针片段，每孔 2 候选取离孔最近 t −15.45 / 3.55；hr33 复审 M-02：壳后拧 F14 的可达性此前未算）；assembly.yaml（`hr35_sync_data.py`）补回 hr34 往返丢的 7 行注释、`head_on_headpitch.present_note` 改"整机 55 件不含髋内轴承/轴套/原版嘴脸"、disassembly_order 新增 seq 0 头子总成 +z 提起（movers = 步 20 的 17 件 + orig_jaw/orig_face_part；counts.disassembly_entries 8）；features H03-F11 `verified_2026-09-22_hr33` 数字口径（±31 起 0.08/0.14、±32 yaw 0 档 0.7/2.3、全档峰 1.9/3.4）；printability parts.H03 `note_2026-09-22_hr33`（8.9 mm²、brim/raft、梳齿）。`build_fast.write_cache` shell_R 修（复审 m-01）。L4 冒烟 vs hr34：F14/F14b 三条路径判据全绿（reach 55.6 峰 0、6/6 可见）、disasm00 0.0、只有 2 条 tool_reach_declared 预期红。

## 2026-09-22 hr34 —— F20 壳螺丝装配顺序：头子总成改在壳之后挂（数据轮，件同 hr33；12:57 起 采样 → Gate）
数据（`hr34_sync_data.py`，PyYAML 往返、文件头注释保留）：assembly.yaml seq 重排 —— 步 14 只留头俯仰舵机进 N01（seq 11，subassembly neck，F13）；步 15 颈上躯干轭只带 N01 + 两颗舵机（seq 12，movers 3 件）；步 16 电池（13）、步 17 壳（14，present 剥掉 19 个头件 → **F20 起子 +z 不再被头壳挡**）；头台架步 11/12/13/18 → seq 15–18（already_installed 补 颈/电池/门/壳）；**新步 20**（seq 19，robot）= 原 14b 头子总成从 +z 落到头俯仰舵机法兰 + F14，workspace body_all、在场 = 整机 55 件；步 19 电子件 → seq 20。tool_states：neck_servo_in_N01 / neck_on_trunk / battery_door_on / shells_on 剥头件，head_on_headpitch → after_step 20、整机在场。subassemblies 注释、counts（steps 20 / verified 11 / declared_motions 19）、disassembly 1/2/6 加 hr34 注释（拆壳前先提头）。L4 冒烟（`gate.py --layers 4` vs hr33 记分卡）：变色 10 条全绿 —— F20 `tool_path_to_outside` FAIL → PASS（reach 51.2、peak 0）、`screw_head_visible` 0/4 → 4/4；step17 `present_only_installed` shell_L/shell_R 与 tool:shells_on 三条旧红转绿（原因同一个：头件在 seq 上还没装却在场）；step20 motion 峰值 1.7e-14、motion_coverage / present_covers_derived / present_only_installed / step_sweep 全绿；0 新红。发布目录 12:55 已同步 hr33 件 + 源码。 **出分 ✅133 ❌382**（hr33 ✅129 ❌385）：预期转绿全部兑现（_release ×2、F20 ×2、step17/shells_on ×3、step20 ×5），0 倒退；step14 的 3 条随动作搬到 step20。

## 2026-09-22 hr33 —— H03 让位刀按策略包络（横滚 ±30）+ grow 0.45（改件；12:1x 起全链，分数见记录 41）
几何：`head.py` 新常量 `HEAD_ROLL_SWEEP = (−30, 30)`（让位刀横滚扫掠范围；`HEAD_ROLL_TARGET` ±25 不改，L6 判 BLOCK/WARN 的边界仍是 frozen 目标区间）、`N02_SWING_GROW` 0.4 → 0.45（消 hr32 的非流形顶点，复审 m-01）。依据：hr32 真实姿态剩 5 个 head_bottom_shell×neck_pitch 碰撞全在 head_roll −26.8..−28.2°（策略实际包络 −28.15..24.04），复审第 3 节第 4 条倾向 (a)。试 build（scratch h03_hr33/test.log）：削 747.5 mm³（hr32 349.8，多 397.7；口袋 6 → 4，合并）；19 个试验姿态全 0（含 hr32 剩的 5 个和 ±30 四角；±32 仍 0.7 / 2.3）；非流形顶点 0；与 H03-F08 转接板口袋交集 0。
数据（`hr33_sync_data.py`）：features `H03-F11` 整块重登（count 4、sweep_range head_roll [−30, 30]、sweep_samples [44, 13]、grow 0.45、4 实例 pos/bbox 全部核过为空 → 都用 bbox_mm、removed 747.38、verified_hr33）；`head_specs.json` head.py 指纹刷到 hr33；`l6_motion.py:79` → `policy_steps_2026-09-22_hr33`。**链模板改序**（`hr33_go.sh`，复审 m-03）：build → slice + merge → 审计 ∥ 扫掠 → 采样 → Gate；build.py 末尾写快车道缓存。预期：L6 head_yaw+head_roll 仍绿、`real_pose_collisions` 里 H03×N02 归零（88 → 83 姿态）、L0 H03 nonmanifold_vertices 绿、L1 H03 first_layer_area 再变（记录）、L2 H03-F11 4 实例绿。F20 装配顺序（拆 seq 16）→ hr34 数据轮。 **出分 ✅129 ❌385**：H03×N02 真实姿态归零（84/39/16）；新判据 nonmanifold_vertices 揭出 H01 ×1 / L05 ×3 旧夹点（WARN，待修）；L1 H03 first_layer 8.9 mm²；发布目录 12:55 同步。

## 2026-09-22 hr32 —— H03 下头壳给 N02 让位（改件：H03-F11）+ hr31 复审数据小修（09:00 起全链，分数见记录 39）
几何：`head.py` 新 `n02_swing_cut`（HEAD_YAW_TARGET (−116.4, 95.4) × HEAD_ROLL_TARGET ±25，5° 网格，N02 minkowski ±0.4，N02 在 H03 系 = A_r(−θr)·A_y(−θy)·N02，逐 θy 并 11 个 θr 副本从 H03 减）；起因：真实姿态 1502/5601 head_bottom_shell×neck_pitch 交叠（走路 1293，峰 55.6 mm³，hr10 起每轮一样）从没进过待修清单（Gate校验_2026-09-22 §一 1）。试 build：削 349.8 mm³（6 块：前下沿 x 39.5..46.5 两块 117/107、后下沿 x −21..10 两块 62/58、|y| 29..35 薄片 3.4/2.5），H03-F08 口袋 / H03-F10 N07 通道 0 交集；16 个试验姿态（目标区间四角/两端/区间外）交集 hr29 9–52 mm³ → 0。
数据（`hr32_sync_data.py`）：features 新 **H03-F11**（sweep_cut，6 实例 pos=块内点 / bbox=块外包盒，export_local）、counts 248→249 / sweep_cut 15→16 / H03 10→11；parts H03 feature_count 11、features_total 249；fasteners **F12b stack 5.55→5.40、engagement 2.45→2.60、孔中点 −15.625→−15.70**（惰轮端面 −13.0，09-14 CHANGELOG 写了没落地，hr31 复审 m-01）；9 组 `stale_since`→`stale_until_2026-09-22`、`tool_access_probe_2026-09-13`→`_was`（m-03）；features T01-F20.geom 重复 depth_kind/through 去一份（m-04）。`l6_motion.py:79` → `policy_steps_2026-09-22_hr32`。
**不改**：F04/F16/F17 stack_mm（复审 M-01：登记值对，L5 只量宿主件一段）；F26（unknown 是设计行为，BLOCK 不可豁免，等非打印件登记方案）；F20 装配顺序、元件占位块 → hr33（先单独看让位对 L6 的影响）。
**hr32 出分（10:10）✅130 ❌384**（= hr31）：转绿 F12b `stack_from_seat_crosscheck`、L6 `head_yaw+head_roll` pair_combination；新红 L0 `_release` ×2（未同步，预期）、L2 `H03-F11:occupancy`（实例 4/5 薄片外包盒中心在料里）。真实姿态 head_bottom_shell×neck_pitch **1502 → 5**（峰 3.14 mm³，5 个都是 head_roll −26.8..−28.2° 超出目标 ±25°）。**链的顺序 bug**：`merge_slice_run` 在采样之后改 `printability.yaml` → 采样指纹作废、`real_pose_collisions` unknown（hr29 同病）。
**hr32b（10:20，件不变）**：features `H03-F11` 实例 4/5 `bbox_mm` → `extent_mm`（不在 L2 BBOX_KEYS，判据点只剩 pos）+ occupancy_note / occupancy_fix 注释；发布目录同步 hr32 24 件 + 源码；`l6_motion.py:79` → `policy_steps_2026-09-22_hr32b`；链改为 采样 → Gate（slice 不重跑，printability 已稳定）。 **出分 ✅131 ❌383**：L0 `_release` ×2 + L2 H03-F11 转绿、0 倒退；`real_pose_collisions` measured 88/5601（hr31 1512；head_bottom_shell×neck_pitch 5 姿态峰 3.14）。 **复审 `hr32_review.md` BLOCKER 0 / MAJOR 2 / MINOR 6 → 收尾**：`head_specs.json` head.py 指纹刷新（M-01）；printability parts.H03 note 贴床 79.2 → 17.3 mm²、切片开 brim（M-02）；features H03-F11 `verified_2026-09-22` 补“让位的 Gate 证据 = L6 pair_combination + real_pose_collisions，L2 只证 6 点无料”（m-04）；`policy_steps_2026-09-22_hr32/run.json` 标注指纹作废（m-05）；`l6_motion.real_pose_collisions` 指纹不一致时点名是哪个依赖文件变了（m-03）；L0 新判据 `nonmanifold_vertices`（m-01，顶点面扇不连通 = 夹点；hr32 H03 有 1 个 @ (41.7, 8.01, 220.59)，hr33 让位刀 grow 0.45 消掉）。

# `tools/gate/data/` CHANGELOG

## 2026-09-22 hr31 —— hr30 复核：F26 的 T01 前壁孔是接收孔（只改数据，几何同 hr29）
数据（`hr31_sync_data.py`）：features T01-F20 kind/check_class screw_hole → **pilot_hole**（+ depth_kind cutter_length / through true / thread_engagement 2.4；by_kind screw_hole 18 / pilot_hole 7）—— 螺纹咬在 3.5 厚前壁里，穿件是非打印的 PCB（叠厚 1.6 记在 F26.stack_mm）；L5『pilot_hole 无头不量』→ 坐面/叠厚/咬入各条转 unknown（PCB 不是打印件，Gate 量不到），不再有 hr30 那两条假红和 seat_* 假绿；fasteners F26 joins 文案、shaft_len_mm / channel_len_mm 115.4（09-09 旧孔位）→ 58.4（hr30 L4 实测 reach 58.316）、seat_measurement_note_hr31、counts 文案 M2×4；tolerances pla_self_tap rule 文案『IMU→T01 前壁 3.5』→ 2.4。真实姿态重采 `policy_steps_2026-09-22_hr31`。Gate hr31 出分（02:55，`gate_full_2026-09-22_hr31.log`）**✅130 ❌384**：对 hr30 FAIL→PASS 12（present 11 格 + L5/F20 engagement_per_hole；F20 tool_origin_on_seat 是新判据 None→PASS 不算转绿）、9 组 tool_reference 红消失（转由坐面判据接手：8 组 tool_origin_on_seat / tool_path_to_outside / screw_head_visible 全绿）；新红 26 条里 9 条 tool_reach_declared（杆长实物未知，预期）、8 条 head_vs_neighbours WARN（判据未实现，预期）；**真发现**：① F20 壳螺丝起子 +z 118 mm 被头壳挡（tool_path_to_outside / screw_head_visible，装配顺序）；② `stack_from_seat_crosscheck` 4 组对不上 —— F04 实测 2.7 vs 记 7.5、F16 2.5 vs 5.25、F17 3.15 vs 5.25、F12b 5.4 vs 记 5.55（**复审 M-01 纠正**：F04/F16/F17 登记值是对的，是多件叠层，L5 只量了宿主件一段 → 不改数，改判据串件量或加归因注释；只有 F12b 是真旧值 5.55 → 5.40）；③ F26 T01-F20 改接收孔后 head_locator 指向的两孔全是接收孔、穿件 PCB 不在 placed/ → head_seat / engagement_per_hole = unknown（记成 FAIL/BLOCK，**就是 part1 预期的**；本段初稿写“转 FAIL 而不是 unknown”是误读，复审 M-02 纠正），hr30 那两条假红、五条假绿都没了。②③ 归 hr32（② 只改 F12b；③ 要么按非打印件完整登记、要么 waivers，zz_imu 占位块解决不了 L5）。复审 `hr31_review.md`：BLOCKER 0 / MAJOR 2 / MINOR 6，hr31 可作 hr32 基线。真实姿态 17/7/7 不变，`real_pose_collisions` 仍 BLOCK 1512 姿态。；**part2（`hr31_sync_data_part2.py`，Gate 校验 §六 B）**：assembly.yaml present 补已装件 —— seq 14 motions H03_bottom_shell / top_head_shell、seq 16 motion neck_servo_into_N01、tool_states H02_on / N02_on_headyaw / head_on_headpitch / head_shells_on / headyaw_servo_in_N03 / neck_servo_in_N01 各加 seq 11 装入的 `bearing_head_yaw` + `bearing_head_roll_A`；tool_states hiproll_flange_L / _R 各加 `hip_pitch_sleeve` / `hip_pitch_sleeve_R`（hr30 L4 `present_covers_derived` 红 11 格；不是采购缺件，是在场清单漏写 → 扫掠少了障碍，可能假绿；预期 11 格转绿、对应 step_sweep / tool_path 可能因障碍变多而新红 = 真结果）；**part3（`hr31_sync_data_part3.py`，Gate 校验 §六 C）**：fasteners.yaml tool_access 坐面重生成 9 组（F02 / F04 / F08 / F10 / F12 / F12b / F16 / F17 / F20）—— 根因：Ø10.5 分度圆定案后法兰孔 r 4.75 → 5.25，09-12/13 探针的坐面没跟着重量 → L4 `tool_reference`（坐面不在孔轴上，1e-6）27 组整体红；probe_seats 对 hr29 件重量，只接受每孔候选面唯一且坐面数 = qty 的组；F06（2 孔无面）/ F14 / F14b（每孔 2 候选）/ 背孔 11 组 / F23 / F25 / F29 / F30（脚本无 SPEC）共 18 组不动、继续红，留 hr32。预期：9 组 tool_reference / tool_origin_on_seat 绿，tool_path_to_outside / screw_head_visible 第一次真跑（可能真红）；tool_reach_declared 仍红（杆长实物未知）；**L4 冒烟（`gate.py --layers 4 --skip-negatives`，scratch）两处补**：① `motion_coverage` 要求 motions[].present 与 required_motion_groups[id].present 逐项相等 → seq 16 neck_servo_into_N01、seq 14 H03_bottom_shell / top_head_shell 的 required_motion_groups.present 同步各加两只头部轴承；② F20_shell_to_T01 feature_hole_map 前两项（T02-F03 / T03-F03）加 `instance: 0`（tool_access.py:75 用它索引 geom.instances 逐孔列表取 axis，先例 F04/F05/F07；两孔 axis 均 z）。冒烟 1：9 组里 8 组 tool_origin_on_seat / tool_path_to_outside / screw_head_visible 全 PASS，只剩 tool_reach_declared（杆长实物未知）；present_covers_derived 11 格全绿

## 2026-09-22 hr30 —— hr29 Gate 两条真红 + 一处漏改（只改数据）+ 用户当日定案（几何同 hr29）
数据（`hr30_sync_data.py` / `_part2.py` / `_part3.py`）：features H01-F17 改 instances 逐实例 bbox（原合并 bbox 中心落在前立板料里 → L2 occupancy 红）；fasteners F26 feature_hole_map 改新孔 (−23.1, ±10.16, −8.35)（hr29 漏改）、补 head_locator_map_indices / tool_access 坐面（T01 前壁外侧面 x −21.1，头朝 +x，工位 tool:imu_install）/ stack 1.6（PCB）/ engagement 2.4，**spec M2×5 → M2×4**（通孔 3.5 墙，screw_length_rule 5 > 1.6+3.5−0.3）；assembly tool_states.imu_install（after_step 9）+ 步 10/19 文案；components camera_csi 定型 Radxa Camera 8M 219（09-22 已下单，envelope datasheet、mass 到货称）、battery 花牌同规格换牌、sbc 已下单、cable_dupont_ff 优信、UBEC 输出改杜邦公对母 40 cm、屏相关/"换香橙派"/USB-C1 供电/IMU SPI 旧文作废标注；harness HB01/HB03/HB04/HB07 尾部/USB-C1/30 cm 舵机延长线/10 cm ×2 旧文改。发布目录同步 hr29 STL + 源码（hr29 L0 source_copy 红）。真实姿态重采 `policy_steps_2026-09-22_hr30`（yaml 进指纹）。Gate hr30 **✅122 ❌392**（hr29 ✅120 ❌393；新绿：L0 `_release` 两条、H01-F17 occupancy、F26 screw_length_rule / head_seat_exists / head_seat_flatness / seat_flatness / seat_stack_spread / counterbore_declared、L4 F26 tool_origin_on_seat / tool_path_to_outside（reach 58.3）/ screw_head_visible、`tool:imu_install` 工位 2 条；新红：F26 `engagement_from_measured_seat` 0.5 / `stack_from_seat_crosscheck` 差 1.9 —— T01-F20 登成 screw_hole 被 L5 当穿件孔量叠厚 3.5，是登记错（→ hr31 改接收孔）；`tool_reach_declared` 缺实物起子杆长（不编，同其余各组）；反例 5 条 L7 旧失败与 hr28b/hr29 同一批，其余 0 变色）。

## 2026-09-21 hr29 —— IMU 定型 Adafruit 4438 LSM6DSOX（STEMMA QT）+ IMU 走线定案 + N07 起子孔 + H01 顶角缺口 + T01 墙-甲板缝加宽（用户确认 7 项）
几何：`trunk.py` T01 前壁 IMU 底孔 2×Ø1.7 → (±10.16, 111.65)（旧 GY-601N1 孔删）、前壁-甲板缝加宽到 4×9（x −21.1..−17.0 × |y|≤4.5 × z 119..121.2）；`head.py` H03 浅槽两道挡边穿 Ø5 起子孔 (58.75, 0, 235.615) x 51.5..66（N07），H01 前立板顶角 2 个 7×4.4 开口缺口（x 46.5..52.5 × |y| 16.8..23.8 × z 257.6..262）。
数据（`hr29_sync_data.py`）：features T01-F20 改孔位、新 T01-F30（cable_channel）/ H01-F17（slot ×2）/ H03-F10（tool_channel），counts 248（T01 30 / H01 17 / H03 10）；parts 三件 feature_count；frozen imu_pose our_mounting_pos (−23.1, 0, 111.65)、delta (−2.1, −0.0664, 6.35)，site 不动；components `imu_icm42688`（id 不改，引用太多）内容换 Adafruit 4438（envelope 25.4×17.78×4.6、1.7 g、SH 侧插、keepout、insert_dir），新增 `cable_qt_dupont_m`（150）/ `cable_dupont_ff`（400，撕 4）；keepouts KO17 实测净空 9.9 零位 / 6.7 扫掠（checked_offline）；harness HB05 4 芯、segment_len [150, 400]、route_polyline（缝→中央通道→KO18→口袋→颈→头→H01 缺口→pin 3/4/5/6）、rules、route_unverified；fasteners F26 M2×5 ×2 @ (−23.1, ±10.16, 111.65)、装 L01 之前；assembly 步 10/19；head_specs.json head.py 指纹。发布目录：多出的 `B02_tail_tray.stl` 移到 `_旧版_hr15_2026-09-19_勿用/`（hr28b L0 `_release` 红的原因）。Gate hr29b ✅120 ❌393。

## 2026-09-21 hr28 —— 转接板进头里（用户定，方案 b：保留 Radxa 排针），B02 尾部托架作废

几何：`head.py` H03 地板下 1.5 壁浅槽（H03-F08 口袋 x 53.8..63.8 × |y|≤21 × z 226..240，槽底外 224.5）托住竖放的转接板（占位 zz_adapter x 54.3..63.3 × |y|≤20 × z 226.8..256.8，挂 jaw_soft），Radxa 排针带（z 258–263）压住板顶；通风槽（H03-F09，09-17 起未登记，本轮补）随槽底下移；`build.py` 不再 build B02；`trunk.py` B01 门顶不再开 B02 底孔。数据：features 删 B02-F01..F04 / B01-F08，counts 重算（245）；parts 删 B02、motion_instances zz_adapter→jaw_soft；fasteners 删 F31；components bus_adapter housed_by H03 + envelope [9,40,30] + 新增 cable_ph20_ext；keepouts KO12 exempt / KO01 unknowns；relations/assembly/printability/bench；harness HB01 根在头里（S1 头→头链 + 延长线→S2 口袋→S3/S4/S5 躯干），HB03/HB04/HB09 路径，链尾压降 0.71→1.11 V、fallback 改躯干就地注电；manifest body_for_part。起因：训练 MJCF（hr12，09-17）不含 B02/转接板，仰躺起立先着地；原版 HAT 在头里叠在 Pi 后。脚本 `hr28_sync_data.py` / `_part2.py`。

## 2026-09-21 —— 线翘 4 mm（hr23 → hr24）：插座让位全部改通窗，颈俯仰单挂

hr24 修正（hr23 全链 ✅117 ❌392 → 复核后改）：wire_clr 0.5 → **0.4**（A 区上沿 −17.4 = L07 6700 座环端面，座环不削）；N02 脸颊**不加厚**（外移 3.05 撞 H03：走路 106 姿态 / 头横滚 −25° 22 mm³）、改为按线翘区扫掠把脸颊下半截挖穿，F14b 回 M2×6 / 叠厚 3.40 咬 2.60（hr26 复审 F-04：09-14 惰轮端面 −12.85→−13.0 后旧值 3.55/2.45 一直没更新，本轮改正）；
T03 让位 grow 0（壁留 1.0）；N01 +y 窗开到板边 + 窗-孔薄环挖掉（N01-F10 min_dirs 22）；KO01 的 **W 只对载体查**（asmcheck / l3_static solid_targets）；sweep_cut 三条的 pos 改切除区内部点。

hr25 修正（hr24 全链 ✅122 ❌393，vs hr22b 只多 1 红 L2 N02-F12:diameter Ø15.58）：N02 `wire_clr` 的膨胀 DILATE6 → **`mink=True` 连续长方体**（`lib.conn_hump_sweep` 加 mink 透传）—— DILATE6 沿毂轴（= 扫掠轴 y）的两个副本不缩 Ø15.6 扣柱孔，A 区端面 −17.4 到脸颊外表面 −17.5 那 0.1 只有 Ø15.6 的孔 → 毂外留 0.1 厚 × 0.3 宽的唇 + 斜向 ≤0.09 薄片（L1 N02 薄区 73.9 → 80.1 mm²）；mink 后毂外干净、多削 5.3 mm³ 皮。`features.yaml:N02-F14` spec/grow_method/removed_mm3 246 → 252 重登（hr23_sync_socket_features.py 加 mink 字段）。

hr26 修正（hr25 全链 ✅121 ❌394：唇没了，但冒出 L0 N02 退化面 2 / 非流形边 2、L2 N02-F12 present 10/24 + bore_gauge 判不了）：① `neck.build_neck_pitch` wire_clr 的 idler_keep **Ø15.0 → Ø14.96** —— mink 后扣柱孔在斜向缩进毂里，扇区内刀的内壁 = keep 柱 64 边形，与毂 64 边形整面重合，导出 float32 塌成退化面；小 0.04 = 刀比毂面深 0.02，扇区里毂 r 7.48（N02-F13 Ø15.0 ±0.1 内，L2 diameter 仍量到 Ø15.0000）。② `features.yaml:N02-F12`（−y 脸颊共用沉坑）判据从 cylinder 整圈壁改 **prism 占位**（Ø15×1.1 必须空；48 边形 r 7.4，box_export_local 给 L1 禁撑区）：脸颊挖穿后坑壁只剩扇形板那 ≈150°，hr24 的 24/24 是 DILATE6 的唇在充数；nominal_d/depth 保留给 L5 螺丝头判据。③ N02-F13 note、N02-F14 spec 跟着重登。预检：`gate.py --layers 0,2 --parts N02` L0 三条绿、N02-F12/F13/F14 全绿。

hr27（hr26 独立复审 `hr26_review.md` BLOCKER 0 / MAJOR 2 / MINOR 10 后的数据与检查体系修正，**几何不变**）：① F-01 发布包 `01_整鸭打印件/` 顶层 24 个 hr15 旧 STL + assembly_preview 归档到 `_旧版_hr15_2026-09-19_勿用/`；`l0_mesh` 加 **`_release:release_root_no_duplicate_stl`**（根目录无 STL；非留档子目录里同名件必须同 sha；名字带 旧版/勿用 的目录不查）。另发现 09-17 退役的 L08/L09/L11/L12 四个 STL 一直留在 cad/duck_s288/ 并被同步脚本通配符抄进 S288专用件/ → 移到 `cad/duck_s288/_retired_2026-09-17/` 与留档目录，`export_vs_release_sha256` 加"发布副本多出（不在 parts.yaml）也红"。② F-02/F-03 `harness.yaml:HB01` S5→5 / S5→6 / 6→7 三跳标 `route_unverified: true` 并写明走廊探针结论（N01 −y 面外侧在 neck_pitch ≥ +30° 被 T03 壳顶压住，线只能走壳外）；`keepouts.yaml:KO01.unknowns` 加两条显式缺口（线翘体不在 L6 组合/真实姿态里；三跳走线未验证），第 1 条"脸颊外移"改口；HB01 链 F why 同。③ F-04 `fasteners.yaml:F14b` stack 3.55 → **3.40**、engagement 2.45 → **2.60**（hr26 件射线 6/6：沉坑底 −16.40 → 惰轮端面 −13.0；09-14 端面改 −13.0 后一直没更新），`features.yaml:N02-F12` purpose/nominal_source 同步。④ F-11 反例 `n_l3_ko01_neighbor_zone.py`：邻件闯 A 必须红 / 邻件只在 W 不许红 / 载体在 W 必须红。⑤ F-12 `head_specs.json` neck.py 指纹刷到 hr26（head audit 0 FAIL）。⑥ F-07/F-10 记录口径改正。未做（MINOR，可选 CAD）：F-05 L07 座环缺口 ≈1 mm² 皮、F-06 N02 脸颊边 4.9 mm² 楔、F-09 wire_out 卡尺值（等实物）。

- `keepouts.yaml:KO01`：dims **wire_out 2.0 → 4.0（measured，用户实物目测：原配线硬、出插头先直立 ≈4）+ wire_clr 0.5**，A 区 x −17.5..−13.0；pocket 型作废；
  rows 9/10（踝）前口 window / 后口 none（L07 后臂 +90° 扫过 37/80 mm³）、row 11（颈俯仰）+y window / −y none（T03 +35° 7.1 / T01 +60° 0.9 mm³）、row 12（头俯仰）两口 window
  （N02 −y 脸颊外移 3.05）；status_detail 23 查 + 5 不查；motion_caveat 改成 asmcheck `socket_motion` 单关节扫掠；unknowns 缺口 y/z 改"按图纸为准（用户 09-21）"。
- `features.yaml`：删 L01-F15/16、L04-F15/16、N01-F11..14（hr16 口袋/板角 8 条），重登 L01-F15/16（板角削，深 4.2）、L04-F15（踝前口窗）、N01-F11（颈俯仰 +y 窗）、
  N01-F12/13（头俯仰两窗）；新增 sweep_cut **L07-F11**（后臂减线翘区绕踝反向扫掠，hr22 件切 73.6 mm³）、**N02-F14**（−y 脸颊减线翘区绕头俯仰反向扫掠 [−92,56.2]，内层通槽）、
  **T03-F08**（壳减颈俯仰 +y / 头俯仰两口线翘区绕颈俯仰扫掠，16.6 mm³）。by_kind window 11→15 / cable_channel 9→3 / sweep_cut 12→15；247→248。生成脚本
  `docs/design_2026-09-17_bearing_rebuild/hr23_sync_socket_features.py`。
- `fasteners.yaml:F14b_headpitch_idler`：M2×6 → **M2×8**（脸颊加厚 3.05，−y 侧沉坑 1.1 → 2.25，叠厚 5.3，咬 2.7；长度规则 8 ≤ 5.3+3.0−0.3 = 8.0 刚好）。
- `harness.yaml:HB01`：topology daisy_chain_5 → **daisy_chain_6_branches**：踝改前口（3→4 零位 44.8）；颈俯仰只剩 +y 口 → 单挂（链 E），头链 F（头俯仰 +y 进 −y 出 → 头偏航 −y 进 +y 出 → 头横滚）另起，
  一分二 **4 → 5（S5 放顶板口袋）**；L_EXIT 2 → 4；worst hop 93.9 ≤ 150。
- `components.yaml:bus_splitter` qty 4 → 5（购物车待改）。
- `parts.yaml` feature_count L04 −1 / N01 −1 / L07 +1 / N02 +1 / T03 +1，features_total 225 → 226。
- 工具：`tools/cad/asmcheck.py` 新增 `socket_motion`（window/free 口线翘区逐单关节 target_range 5°/档 与不同 body 件求交，--from-placed 时跑）；
  `tools/gate/layers/l3_static.py` KO01 加 wire_clr_mm。

## 2026-09-20 晚 —— 电子件安装位（hr17/hr18）：B02 尾部托架、T02 开关面板、T01 扎带环登记

- `parts.yaml`：新件 **B02**（tail_tray，底板朝下）；`motion_instances` 加 tail_tray / zz_adapter（转接板占位）→ trunk_base；counts.parts 24 → 25。`printability.yaml:parts.B02`。
- `features.yaml`：frames.per_part.B02；hr16 插座让位 16 条 prism void（T01-F27/28、L01-F15/16、L03-F07..10、L04-F15/16、N01-F11..14、N03-F11/12，`hr16_sync_socket_features.py`）；hr18 加 B02-F01..F04（过孔/口袋/底板/扎带槽）、B01-F08（门顶条 Ø1.7 底孔）。
- `fasteners.yaml`：**F31_B02_to_B01** 2×M2×6（穿凸台 3.5 进门 2.5，尖端离门内面 0.5）；坐面探针/feature_hole_map 未登记（L5 红，待 probe_seats）。
- `assembly.yaml`：步 16 带 B02（门+托架一起 +x 推入；required_motion_groups 与 motions 的 battery_door movers 都加 tail_tray/zz_adapter；parts 加 zz_adapter；所有含 battery_door 的 present 补两项）；拆序 4 movers [B01, B02, zz_adapter]。
- `keepouts.yaml:KO12` exempt 加 B02（抽出走廊段，随门拆；仓腔段不豁免）。
- `relations.yaml:parts_without_declared_load_path` 加 B02（只托转接板，不在传力链上）。
- `components.yaml`：bus_adapter housed_by B02 + claim placed_prefix→zz_adapter + envelope_solid 30×40×9；switch_rocker 定位 T02 壳顶（KCD1-101 21×15×20）；buck_12v_5v → 航模 UBEC 5V/3A（T01 −y 侧腔）；新增 connector_wago_221_412 ×5；display_lcd qty 0（09-19 取消）。counts 23 → 24。
- `harness.yaml`：HB03（12 V：电池→开关→转接板）/ HB04（5 V：拓展口→UBEC→40 针）走线按新位置重写。
- `tools/sim/cad_geometry_manifest.json`：body_for_part 加 tail_tray / zz_adapter（漏了它 hr17 第一次跑运动审计/真实姿态开头就炸）。
- hr17b Gate ✅114 ❌404 → hr18 见 `docs/design_2026-09-17_bearing_rebuild/记录.md` 第 25–26 条（新红逐条归因：未实测/未探针/组合极限）。

## 2026-09-20 —— S288 插座模型改为背插（hr16）：KO01/HB01 重写，串联 5 条链

- 事实：用户舵机到手拍照，两个 PH2.0 座在**背面（惰轮面）接线端两角**的缺口里、针朝 −x、插头沿 x 从背面插，插到底与厚段 23 面齐平，只有线再凸 1–2（口径 23–25）。09-11 拿手册尺寸图逐像素反推出的"侧插（±y）/ 10 颗只有 1 口 / Y 线分接"整套作废（docs/design_2026-09-17_bearing_rebuild/电子件尺寸核实_2026-09-19.md §8）。
- `keepouts.yaml:KO01` 整条重写：实体 = A 插头顶/线弯区（x −15..−13 × |y| 5..10.5 × z −14.4..−4.2，扣 Ø15.6 惰轮毂柱）+ B 侧出线槽（|y| 10..13.3）/ W 背板通窗（x −18.5..−13）；逐颗逐侧 `mode_pos_y/mode_neg_y` ∈ {pocket, window, free, none}（与 `duckstructure/lib.py:CONN_MODE` 同表）。dims 新键 plug_top_x/wire_out/plug_clr/plug_y_in/side_exit_y/window_x/hub_keep_d；旧 window_face_grid/probe_offset、socket_center、recess_depth、plug_protrusion、pull_clearance、cad_fix_required CF1–CF5 删（历史进 geom_superseded）。status checked_partial → checked。
- `tools/gate/layers/l3_static.py` KO01 段按新 dims 建实体（查全部打印件，none 侧不查；CAD `conn_cut(R, sides, mode)` 只作对照，同侧 A+B/A+W 并起来比）；`window_face` 判据随侧插模型一起退役。反例 `n_l3_ko01_independent` 改成：声明通窗加深到 −25 而 CAD 只切到 −18.5 → 独立实体 64 mm³ 红；pocket 模式侧出线槽里的料红、同料 window 模式绿；dims 缺 unknown —— 通过。
- `harness.yaml:HB01` 拓扑 bus_tree_y_tap → **daisy_chain_5_branches**：转接板 PH 座 → 4 根一分二（S1..S4）→ 左腿链（髋横滚→髋俯仰→膝→踝）/ 右腿链 / 左髋偏航单挂 / 右髋偏航单挂（外侧口压在冻结壳柱下，只有 1 口）/ 颈头链（颈俯仰→头俯仰→头偏航→头横滚）；每跳 1 根舵机原配 150 mm 壳-壳线（14 用 1 备），零位跳距 19–63、估全行程上界 +9；pin_order 转接板侧实测 SIGNAL–VCC–GND（B 版）；供电全经 PH 座（宇树设计），A 口径 4.48 A > 2 A 额定 → BN02 实测定，fallback 分链注电保留。
- `components.yaml`：`bus_adapter` 四口位置与公母（09-20 实测：PH 座 + XT30(2+2) 公一端，XT30 入口公 + Type-C 一端，均短边侧出）、拟装尾部模块；`cable_ph20` 改为原配线 15 根 + 新增 `bus_splitter`（一公分二母 ×4）；`servo_s288.pinout` 加转接板侧实测旁证（v 仍 null 到 BN16）。counts.components 22 → 23。
- CAD（hr16）：`s288.py` conn_x/plug_top_x/wire_out/plug_y_in；`lib.py` CONN_MODE/conn_zone/conn_cut；L01 削板角、L03 四口通窗、L04 两口口袋、N01 四口口袋、N02 −y 脸颊线弯区 ±45° 扫掠让位槽、N03 两口通窗（裙墙不再开口）、H01 去掉头横滚刀与 CF3 走廊、H03 去掉可见孔、T01 顶板内侧口通窗。`tools/cad/asmcheck.py` 插座段按 CONN_MODE 查 A/B/W 对全部 placed 件。
- `assembly.yaml` 步 12：偏航舵机 CF3 插头改为装好后从背板通窗插（"先接好"不再必要）。

## 2026-09-18 凌晨 —— Gate 三次全量后的装配数据修正 + Gate/采样并行

- `assembly.yaml`：run1 后补 27 处 `present` 缺项（新轴承/轴套/头部四新件）；run2 后步 6 parts 去掉 6703/L10（轴承+轴套不与舵机同装），步 8 新增 `L/R_pitch_bearing_sleeve_into_L03`、`L/R_roll_bearing_sleeve_into_L01` 4 条推入动作（present = 全步在场集），`hip_onto_roll_flange` movers 加俯仰轴承+轴套；步 14/15 头部新件加入 movers（随头动）；`tool_states.headroll_flange.after_motion` → `h_roll_servo_into_N03`、present 改 N03 台架在场 9 件。步 11 d（N06 进腔）保持 known_fail（真几何，hr13）。
- `tools/gate/gate.py --jobs`、`tools/sim/policy_pose_collisions.py --jobs`：按层/按姿态段并行（记录.md 18），结果与串行逐格/逐行相同；L6 `_REAL_POSES` → `policy_steps_2026-09-17_hr12b`（并行重采样，poses sha 与 hr12 相同）。
- run4（并行，10.8 min）后再修：步 8 `6704×1`→`bearing_left_knee/right_knee`、步 11–13 `6704(yaw)/(A)`→显式实例（型号→外径解析会把头部 6704 算进腿步，present_covers_derived 假红）；4 条轴承+轴套推入动作加 `subassembly_of`（俯仰 upper_leg / 横滚 trunk）+ 物理正确的 present（L02 不在场）；横滚推入 len 20→60、N07 12→40（末端脱离证明）。**L4 step08 PASS**；step11 只剩 N06（真几何）。
- 记分卡：run3 ✅112 ❌398（`docs/gate/gate_full_2026-09-17_hr12.log`）；并行全量最终 ✅108 ❌402（`docs/gate/gate_full_2026-09-18_hr12_parallel.log` RUN 4：`tool_states.hippitch_flange_L/R` 补俯仰轴承+轴套；L6 `_REAL_POSES` → `policy_steps_2026-09-18_hr12c`，采样指纹有效，`real_pose_collisions` 从 unknown 变为实判 1514 姿态 / 17 对 = 之前 unknown 那 6 格转为实红；相对 run1 无任何格由绿转红）。

## 2026-09-17 夜 —— 轴承恢复 hr12 收尾：豁免绑 STL、L6 指 hr12、6703 质量、头部装配顺序按新序重写

- `waivers.yaml` 第 3 条：绑上 `cad/duck_s288/L03_upper_leg.stl` / `T01_trunk.stl` 的 sha（hr12 build）；hr12 真实姿态对比该 body 对 1 姿态 / 0.099 mm³，与豁免一致。
- `tools/gate/layers/l6_motion.py` `_REAL_POSES` → `tools/gate/out/policy_steps_2026-09-17_hr12/summary.json`（placed 指纹已核）。
- `components.yaml` `bearing_6703zz.mass_g` 4.2 g（目录净重；实心环 5.92 g × 71%，与 6704 的 72.7% 同量级；到货称重替换）；`bearing_6704zz` 加 `stale_keys_note`（取消口径的旧键只作历史）。`tools/sim/make_mjcf.py` 髋横滚/髋俯仰轴承按 6703 计，四份 MJCF 重出（整机 762.7 g）。
- `assembly.yaml` 步 11/12/13 **按 hr11 新顺序整体重写**（不再只是 note）：步 11 = N03 台架子总成 10 个分动作（偏航 6704 → N08 → N02 轴颈 → N06 → B 22×16×4 → A 6704 → N05 → 横滚舵机 → N04 → N07），步 12 = 偏航舵机 −y 横进 + F15/F16，步 13 = H01 从上套下 + H02（F18/F19）；present 按累计在场（F-L4-1），方向/长度来自 mechanical_audit paths + 全序累计扫掠（14/15 条 0）。
  **已知红**：步 11 `d_N06_into_cavity_onto_journal`（N06 Ø21.6 抬 4–5.5 后沿 −y 出腔口与 N03 交叠 ≈5 mm³）——N06 进腔路径以前从未检查过，hr13 在 neck.py 给 N03 减掉该扫掠。`subassemblies.head` note 同步。
- `fasteners.yaml` F29/F30：删掉从 F19 抄来的 `tool_access_probe_2026-09-13` 与 pilot 块（Ø1.6/4.5/H01-F12），改为 N04/N03 耳的 Ø1.7 / 5.4 / 5.3（design）、`drive_direction` 改正（+x 经 H01 前板孔 / 下方 −z）。
- 发布目录 `01_整鸭打印件/S288专用件` 同步 24 件（L0 `export_vs_release_sha256`）、`06_CAD源码/duckstructure` 同步；打印清单重写为 hr12 版。

## 2026-09-17 —— 轴承恢复 hr10/hr11：装配顺序改写、F06 平垫、髋两站不加压盖（用户拍板）

- `assembly.yaml` 步 11/12/13：加 `hr11_order_note`（不改原字段，`verified: false` 不变）：头部新顺序 = N03+N05+A 轴承+横滚舵机(F17，腔空)+N04/B 轴承/N07 →
  偏航舵机 **−y 横着滑进** N03 → **H01 整个从上套下**（H01 去掉笼底板 `lower`）→ H02 → N02/N06/F16 从下 → H03。依据 `docs/design_2026-09-17_bearing_rebuild/记录.md` 15。
- `fasteners.yaml` F06：加 `washer_2026-09-17`（L02 俯仰盘 180° 那颗坐面 61/72，加 Ø4.5×0.5 平垫 ×2，采购清单同步）。
- `waivers.yaml` 新增 WARN 豁免 `L6/L03:real_pose_collisions:trunk_base×upper_leg_left`：1/5601 姿态 0.10 mm³（左髋横滚 25.5° > 关节区间 ±22°；修它要把 6703 座壁削到 <1.1）。用户 09-17 拍板接受不改件；STL sha 等 hr12 补绑。
- `tolerances.yaml:servo_output_bearing`：宇树客服 09-17 17:29 答复「输出端轴承，另一个侧是衬套」→ 记 support_reply，v 仍 null（无额定值）；问题清单同步。
- 髋横滚/髋俯仰 6703 外圈 +x 侧不加压盖：座唇 + 舵机堆叠定位（与膝 6704 同），BN08 装配时量轴向旷量，>0.2 mm 补厌氧固持胶（记录.md 15）。

## 2026-09-09 —— 给 Gate 第 2/5 层补机器可读几何（数据修补，不动检查器）

改动只落在 `features.yaml` / `fasteners.yaml` / `relations.yaml` / `tolerances.yaml` + 本文件。
12 个 YAML 全部 `yaml.safe_load` 通过；逐键对比修改前后，**原有字段一个没丢**，只有下面明确列出的 11 处原字段值被改。

### 1 · `features.yaml`（缺陷 ①②⑧⑨）
- **新增顶层 `frames:`**：`export_local` 的定义（= `checks.py:17-19 export()` 的 `to_local`，即 `inv(TW(body))·p_world`）、
  三种旧 `datum_frame` 到它的变换、17 个件的 `world_to_export_local_R/t` + 导出 STL 包围盒、19 个舵机帧的
  `servo_to_export_local_R/t`，以及 8 个件的射线核验结果。
- **151 条特征全部新增 `geom:` 段**（只增不删）：`shape / nominal_d_mm / depth_mm / depth_kind / through / axis / pos /
  axial_span_mm / bbox_mm / hole_positions_mm / count / pitch_r_mm / frame / frame_note / derived_from / confidence`，
  坐标与轴向**全部改写到 `export_local`**。数值全部从 `duckstructure/*.py` 与 `tools/cad/ankle_split.py` 抠，不从 `spec_verbatim` 反推。
- `confidence`：`exact` 126 / `derived` 23 / `unresolved` 2（`L06-F04` 鞋底 4 孔、`H03-F02` H01 四脚接收孔，
  两者都是 `not_modeled`：原版网格里的孔，代码没建模也没量过，**不编**）。
- **53 条特征做了导出件射线核验**（`verified_on_export`，读 `cad/duck_s288/*.stl`，沿轴 17 站扫描），全部通过，
  这些条目的 `nominal_d_mm.src` 因此升级为 `measured`。另有 12 条标 `pass: null` + 原因（法兰让位开在自由端面、
  环形刀轴心被毂占住、凸台埋在实体里 …），**不当反证**。
- 缺陷⑧：8 条 `horn_hole` 全部补 `pitch_r_mm = 4.75 (measured, s288.py:19)`（原来只有 `L01-F04` 在自由文本里写过）；
  60 条孔类特征里 57 条补齐 `through` + `depth_mm`（`depth_kind` 区分刀长/盲深/实体厚度），剩 3 条见上面的 unresolved / 多实例条目。
- 缺陷⑨：8 条 `horn_hole` 各加 `clock_phase: {declared_deg: null, datum: null, unknown_class: not_selected,
  must_declare_before: 装配, blocks_relation: R23}` —— **代码里确实没有相位约定，没编**。

### 2 · `fasteners.yaml`（缺陷④）
- **30 组全部新增 `feature_ids` + `feature_hole_map`**（含每颗螺丝在 `export_local` 里的孔坐标）。
  29 组有孔坐标；`F23_sole_to_foot` 没有（唯一目标 `L06-F04` 未建模）。
- N01 那 6 个背孔按孔位拆清楚了：`F11` = `N01-F06` 上排 2 个（索引 0/2）、`F11b` = `N01-F06` 下排 2 个（索引 1/3）、
  `F13` = `N01-F07` 的 2 个，三组互不重叠。
- 8 个自攻组新增 `pilot` 段（`pilot_hole_d_mm` / `pilot_depth_mm` / `declared_in_feature`），
  从 `joins` 自由文本抠出并与源码逐条核对；`F22`/`F23` 保持 `null` + `not_modeled`。

### 3 · `relations.yaml`（缺陷③⑨⑩）
- 缺陷③：`R09` → `engagement_by_joint_type.s288_metal_thread`；`R10`/`R20` → `engagement_by_joint_type.pla_self_tap`；
  `R12` 指的 `fits.axis.concentricity` 桶本轮在 `tolerances.yaml` 里真的建出来了（区间引用已存在的 `datum_vs_mjcf_mm`，没新编数）。
- 缺陷⑨：**新增 `R23`（舵盘零位相位未声明，status=open / BLOCK）**，会一直判红直到有人定下基准。
- 缺陷⑩：`counts` 补 `open_definition` + `open_relations / open_gate_entries / open_total(19) / ok_relations` 与各自的 id 列表。
- **改了原字段值**：`counts.relations 22→23`、`counts.open 15→16`（因为加了 R23，必须与列表长度自洽）；
  `R09/R10/R20.target_ref`（悬空→实际键）；`R18.between[1]` `2.6×2.4×3.4`→`2.3×2.4×3.4`（x 向记错，见下）；
  `R06.note`/`R10.note` 各追加一句指针。

### 4 · `tolerances.yaml`（缺陷③⑤⑥⑦）
- 缺陷⑤：`fits.pla.plain_slide_head_roll` 拆量纲 —— 原 `nominal_mm`(直径) 与 `target_range_mm`(单边间隙) 都保留并各加
  `*_quantity`，新增 `nominal_d_mm` 与 `radial_clearance_mm`（discA/discB 分开，名义值与实测值分开），
  并把 `conflicts.seat_d` 那句"必须按 0.09 判"变成机器可读的 `gate_must_use`（discA 0.09 / discB 0.29 + `verdict`）。
- 缺陷⑥：`fits.snap.b01` 新增 `compare_field: protrusion` + `compare_value_mm 1.0 (measured)` + `compare_note`。
- 缺陷⑦：长度公式的唯一权威定义留在 `screw_length_rule`（新增 `authoritative: true` / `tip_margin_mm` /
  `blind_depth_by_joint_type`）；`engagement_by_joint_type.*.rule` 两条**改写成引用**（原字段保留，只改值）。
  `pla_self_tap` 另加机器可读的 `violations / satisfied / unknown`。
- 缺陷③：新建 `fits.axis.concentricity`；`placeholder_summary` 计数 21→22 并补上该桶。
- 新增 3 条 `conflicts`：`deck_keyhole_clearance`、`L01_hiproll_counterbore_d`（见下），`seat_d` 补 `gate_must_use` 指针。

### 5 · 抠代码时发现 `features.yaml` / `relations.yaml` **原本就记错**的条目（全部已登记，未静默改判据）
| 条目 | 原记 | 实际（源码 + 导出件射线） |
|---|---|---|
| `L05-F07` | 2×Ø4.6 起子通道属于 **L05** | 属于 **L07**：L07 STL 在舵机帧 x −19..−17 处 36/36 向命中 r=2.2973；L05 整个 x 区间量不到 |
| `L01-F09` | 沉孔 Ø4.2 深 1.25 | 有效沉孔是原版自带的 **Ø4.84**（36/36 向 r=2.4200），我们的 Ø4.2 刀全落在里面，不产生新坐面 |
| `T01-F20` / `F26` / `R10` | IMU 底孔"深 10" | 前壁只有 **3.5** 厚且孔直通电池仓 → 自攻咬入上限 3.5 < 2×d=4.0，`pla_self_tap` 不满足的是 **3 处**不是 2 处 |
| `T01-F16` / `F21b` | 仓门底孔 count 2 | x=−44.5 那颗**没有接收料**（24 向只 5 向命中），已加 `effective_count: 1` |
| `T01-F09` / `R21` / `fits.deck.keyhole` | 甲板钥匙孔单边 2.35 | 成品件上被载体前板堵住，有效孔是 Ø14.9 滑槽，单边 **0.49**（z 119.2..121.2 全程 r=7.441） |
| `R18` | L07 榫槽 2.6×2.4×3.4 | **2.3**×2.4×3.4（`ankle_split.py:49` −13.6−(−15.9)=2.3）；间隙 0.2 的结论不变 |
| `T01-F22` | 壳柱高 8.9/10.7 | 那是**左半壳**两根；右半壳是 9.1604/10.8431（`shell_inner_z` 逐根射线量） |
| `L05-F08` | 扫掠 ±37 n75/n11 | 现行代码是 **±62**，舵机包络 n=125、小腿 n=21（`lib.py:24`） |
| `conflicts.seat_d` | 壳座 r10.89 | 导出 H03 半座最小内半径 **10.9000**（间隙 0.10/0.30）；仍按 0.09/0.29 保守判 |
| 多条 `datum_frame` | `part_local` / `world_verbatim` | 实际是舵机帧（如 `L01-F01/F07/F09`、`L05-F07`）；逐条写进 `geom.frame_note` |

## 2026-09-12 —— Gate 清红第一批（检查器与数据，未改 CAD 几何）

- `parts.yaml` 成为 52 个 placed 实体运动归属的唯一声明；补入 N04、头横滚 B 轴承、左右髋偏航轴承。18 件逐项列出 `placed_instances`，按 BOM qty、唯一性与文件存在校验，修复 `rstrip('_R')` 导致的不确定左右查找。`cad_motion_check.py snapshot` 已刷新；快照只记录身份，不能代替运行检查。
- L1 禁撑孔支持 `positions_mm`，嵌套组明确 `count_kind`，组数/孔数/总柱数均校验；缺组、缺孔、重复组、不有限坐标不能缩小检查全集。补 B01-F04、H01-F09、T01-F16/F19/F20 轴向段；69 条禁撑特征中 66 可定位，余 L01-F13/L03-F05 是非柱形装入廊道，L06-F04 未量，继续失败。
- H03-F02 四孔身：432 条径向射线，Ø2.1999937245..2.2000034450 mm；前三总盲深 10.0（含 0.7 扩口），第四底口通壳腔，无统一盲深。新增 H03-F04 的四段 Ø2.8×0.7 扩口禁撑。L5 `self_tap_radial_overlap` 得 F22 `(2.0−2.2)/2=−0.1 mm`，明确为自攻不咬合，不能说补数据后就没问题；未改孔或换螺丝。
- 修正 KO06/KO07/KO09/KO11 的坐标帧声明。未通过改实体或放宽阈值消红。
- L4 的 1/2/3/4/6/7 共 6 步、14 条刚性子动作，分左右工位，固定移动组、在场全集与抽出向量；独立 `required_motion_groups` 防删路径/删障碍。0.25 mm 步距，含精确终点，14 条逐件 STL 探测共 7,260 次证据，最大交集 0.0004162917 mm³。该数不证明连续运动或整机拆卸可行。
- 起子检查迁到 `tool_access.py`：头侧孔身份、真坐面、轴向、足印与拧紧工位均显式声明；批杆 `bit_d_mm`、可用杆长 `available_shaft_len_mm` 与旧通道/需求下界分开。缺声明的其余 27 组判 unknown，取消从孔中点/沉坑直径产生的假几何结论。F01/F24 共 8 坐面 1,152 次足印内外探针全过；工具峰值分别 0、1.58e-15 mm³，可用杆长仍未知。
- F01 旧 129.67 mm³ 实际撞未安装的头壳，不能解读为“被自己的孔挡”；F24 的旧世界 −x 实为舵机局部 −x，左世界 −y/右 +y。新增刀路只证明相应工位安装，整机维修拆卸仍独立判。
- 六个新反例模块覆盖上述漏检；旧 L4 反例只补新的 BOM/真坐面/工具规格声明，保留原始坏几何。22 个反例全过；运动检查器 18 项 unittest 全过。独立复审两轮发现的 MAJOR 全修，已审范围 0 BLOCKER/0 MAJOR；这不代表整机放行。
- 完整 Gate 结果与后续待办见 `tools/gate/out/scorecard.md`、`docs/README.md`；运行日志 `docs/build_logs/gate_2026-09-12_cleanup.log`。分度圆停线、所有实机未知和冻结项仍保留。
- 完整 Gate 首跑 1795.2 s：76 PASS / 363 FAIL / 1 STALE / 7 NOT_RUN，2073 条判据中 915 FAIL；401 条红无测量（旧为 375），不是物理缺陷减少。相对交接的 70/378：6 个装配步骤完成验证，新增覆盖门槛；4 个旧 L4 绿格因缺真坐面改红，9 个旧起子错误归因的件级格子撤掉。所有 140 个运行输入前后 hash 一致；无 NaN/Inf 测量、无 evidence_n=0 的 PASS、无层执行异常。
- 随后补 L2 镜像数值防护：单方向 NaN/Inf 或无采样点必须失败，防 `max(有限值, NaN)` 隐藏失败；新反例正常双向距离最大 2.52e-15 mm，NaN/Inf/空方向均拒绝。仅复跑 L2（依然先跑全部反例），其余层沿用已绑定 hash 的完整运行结果。
- L2 正式复跑完成（`docs/build_logs/gate_2026-09-12_cleanup_l2_finite.log`，1239.7 s，含全部反例）：**23/23 反例通过**，605 条 L2 判据 / 187 条红；最终汇总仍为 76 PASS / 363 FAIL / 1 STALE / 7 NOT_RUN、2073 条判据 / 915 FAIL（401 条无测量）。退出码 1 表示整机 BLOCKED，不是执行崩溃。
- 最终复核：7 对镜像各 2400 点有效，18 个 STL 洞边全 0；137 个格子输入文件 hash 全部匹配磁盘，无非有限测量、零证据 PASS 或层执行异常。`L6/_joints` 唯一 STALE 是主动撤销“必须等于上游限位”的权威，未改限位；其余未重跑层以输入未变沿用。独立复审第 3 轮确认 L2 防护范围内 0 BLOCKER/0 MAJOR。

## 2026-09-12 —— Gate 清红第二批（进行中，用户中途叫停；详见 docs/交接_2026-09-12_Gate清红.md §8）

- L4 `judge_fixed_path` 支持 `segments` 多段直线（首尾相接，终点脱离证明用最后一段方向）；反例 `n_l4_segment_end_inside`。
- assembly.yaml：步骤 5/8/9/10/11/12/13/14/15/16/17/18 共 36 条 `motions` + `required_motion_groups`，`tool_states` 24 个；step 6 工位改为 L01 已在躯干上；step 9 重写为偏航舵机侧滑（旧"腿子总成"与 F02 从杯腔拧矛盾，作废）；step 18 记录原版上/下头壳合缝相交 67.894（冻结件，H03 侧让位已进源码）；step 19 标 not_modeled。**尚未跑 L4 正式扫掠。**
- features.yaml：L06-F04 实测导出网格 0 孔（absent_in_model）；L01-F13 / L03-F05 新增 `box_export_local`（servo_slide 实体外包盒）；被 dump 吃掉的 rib_probe 说明注释移到文件头。fasteners.yaml：F23 status=no_geometry；头部注释补回。keepouts.yaml：KO07 顶 224.3→224.26（=ZF−NP_DISC_T）；KO01 说明注释移到头部。
- L1：`_feature_boxes/_box_section` 盒形禁撑区；`min_wall_sliced` 改用 `layers_single_bead`（Arachne 双 bead 墙不再误判单圈；旧记录缺该字段 → unknown）；反例 `n_l1_support_in_box_zone`、`n_l1_single_bead_gcode`。`slicing/slice_l1.py` 解析修正（;TYPE: 跟挤出记层；外圈走线长/截面周长比值）。slice_run.yaml 只重切了 L02/L03，printability.yaml 尚未更新。
- 新工具 `tools/gate/probe_seats.py`（螺丝真坐面探针）。tool_access 27 组仍 unknown，探针后台未完。
- duckstructure 源码已改未 build：neck.NP_KO_OVER、head.KO_OVER/通道刀最后一刀、head H03 填料+Ø1.7 底孔、head HEAD_SEAM_CLR、legs L02 俯仰侧填柱 y 34.0..40.3。

## 2026-09-12 晚 —— build #7 + 参考复刻标定（docs/参考对比_2026-09-12_AI-FanGe复刻.md）

- `duckstructure.build` #7：上批源码改动（NP_KO_OVER / KO_OVER / H03 填料+Ø1.7 底孔 / HEAD_SEAM_CLR 0.1 / L02 填柱退到板内面）已出 STL、placed/、audit_summary.json、mechanical_audit.json。静态 0；`[已修接口与错相回归] PASS`；H03×原顶壳合缝 67.898 → **0.0 mm³**（`H03_original_seam.current_mm3`），H03×舵机名义间隙 0.29999；退出码 2 仍是冻结的 B01 x=−44.5 后孔未闭环 + 扫掠 268 对（同前）。三个只读检查 idlercheck / asmcheck / audit_screws 均 exit 0（`docs/build_logs/checks_2026-09-12_build7.log`）。`tools/cad/mechanical_audit.py` 的合缝检查改为容忍零体积交集（交集退化成贴合面片时不再做布尔差，直接记 0）。
- **发布目录未同步、18 件未重切片、Gate 未全跑**：PrusaSlicer 的 dmg 挂载点（/tmp/microduck-slicer-check-20260908）随关机丢失，重切要先把 2.9.6 再挂回来。
- 标定：用 L1 `min_wall_geometric` 的原判据判 16 件已实打实打出来的原版件（AI-FanGe 复刻 3MF 的朝向）→ ~~17/20 红~~ **11/16 红**（09-13 复审发现第一版朝向轴系错了，`orient_icp.py` ICP 配回原版坐标后重算；结论不变）。该判据降 BLOCK→WARN（`l1_printable.py`），证据与不改阈值的理由记在 `printability.yaml:criteria.min_wall_perimeters.calibration_2026-09-12`；`min_wall_sliced`（真 G-code 单 bead 层）仍 BLOCK。反例 n_l1_thin_wall / n_l1_single_bead_gcode / n_l1_support_in_box_zone 复跑通过。
- 措辞修正 8 处："髋偏航副支撑原版没有" → 原版 `robot_walk.xml` 在髋偏航轴上本来就有 22×16×4（舵盘面平面 z 115..119；受力报告 §4 判为推力式夹持），S288 配不了那种夹法才改法兰侧径向 6702（README、legs.py、lib.py×2、trunk.py、mechanical_audit.py×2、受力分析报告表格）。几何未动。
- 待用户定：M2 沉孔 Ø4.2 比原版 Ø4.4/4.84 紧，叠 TC02 实测缩水 0.15/边 → ≈3.9 < 盘头 4.0；靠切片补偿则够（单边 0.1）。`fits.pla.m2_counterbore` 仍 placeholder。
- 原版孔径普查（`docs/probes_2026-09-12/参考对比/holes_orig.json`）：M2 自攻底孔原版用 Ø1.6，与我们 Ø1.6/1.7 一致；原版底壳 Ø2.2 是过孔（配 Ø4.4 沉）而非自攻底孔——H03-F02 "不能成牙"的根子在此，09-12 填料+Ø1.7 已按原版惯例修正。

## 2026-09-13 —— 复审修订 + 『有则改之』（用户拍板）+ L1 标定门禁

- 独立复审（workflow verify:doc）抓出 参考对比 文档 4 处硬错：§2.1 打印朝向轴系错（OBJ 原始坐标 ≠ 原版网格坐标）→ 标定重算 11/16 红；§5-B "309 mm³ 重叠"错（annulus 放低 2 mm，实为 0 干涉、推力式夹持成立）；§5-C/D 原版头壳 Ø2.2 是 9 mm 深 M2.5 底孔（配 motor_support Ø2.7），不是过孔；"原版没有"措辞漏 7 处（components/features×2/relations/parts.yaml、assembly_audit.py、6702 实施规格、probes/audit_block.py）→ 15 处全改。文末有修订清单。
- **过孔 2.2→2.4、头窝 4.2→4.4**（用户 09-13：有则改之，不靠切片补偿）：`s288.py horn_hole_d/mnt_hole_d`，`lib.py P.m2_cbore_d / P.tube_hole_d`（driven / horn_cbore / mount_cut_local / mnt_cut2 / head H02 / neck 脸颊沉窝 / NP_CB_D 13.8→14.0 全引用它），`trunk.py` B01 门孔，`tools/cad/ankle_split.py through_d/counterbore_d`。T01 扎带孔 Ø2.2 未动。Gate 声明：新工具 `tools/gate/retarget_hole_d.py --apply` 改 features.yaml 40 处 nominal_d_mm（文本级，注释保留；跳过 T01-F19 扎带孔、H03-F02 旧底孔）；`tolerances.yaml` fits.pla.m2_through_hole 2.4（2.35..2.55）/ m2_counterbore 4.4（4.3..4.6）、`process._positioning` 口径改为"内轮廓按实测让、外轮廓等 TC01b"；`components.yaml` 两处 src_note。**未 build**，等分度圆定案（TB02 一扣）后与 horn_r 一起 retarget + build。
- **L1 标定门禁**：`printability.yaml:calibration_set`（8 件原版网格 + ICP 朝向 + max_block_fail_fraction 0.5 + checks_in_scope 8 条几何判据）；`tools/gate/calibrate_l1.py` 跑真 L1 出 `data/l1_calibration.yaml`（指纹 = process/criteria/materials/calibration_set 规范 JSON + l1_printable.py 源码；逐件 mesh sha）；L1 末尾 `_calibration/proven_parts`（BLOCK）：记录缺/指纹不符/网格 sha 不符/范围未声明 → 红；范围内任何 BLOCK 判据失败比例 > 0.5 → 红（unknown 不计分母，print_orientation/slice_freshness 是簿记，范围外只报不判）。当前记录 2026-09-12 23:35：min_wall_geometric[WARN] 5/8、support_reachable[WARN] 4/8、BLOCK 判据 0/8 → 绿。反例 `n_l1_calibration_record.py` 6 例（误伤红/干净绿/过期红/缺记录红/换网格红/缺范围红）<1 s 全过；n_l1_thin_wall / single_bead / support_in_box_zone / core×4 复跑通过。

## 2026-09-13 —— Gate 数据对表 build #7（I3/I4/I5/I6 的 side_effects；I1 未实施故不做）

只改数据（features / fasteners / tolerances / parts / assembly 五个 YAML，文本级，注释未动），不动检查器、不动 duckstructure、不动 printability/slice_run。每个数要么 build #7 STL 射线/trimesh 实测（写 sha256 前 12 位），要么源码常量（指到 head.py/neck.py/legs.py 行）。
- features.yaml：N02-F05 nominal 15.0→15.05（neck.py:86+90，36 向 r 7.516..7.525，STL e283d7f6a23c），depth_kind blind_depth→cutter_length（下端开进原版顶盒空腔，L2 掏空 8.64≠刀长 9.265，pre-existing 红）；H01-F08 大通道 4.6→4.65（head.py:9+101，r 2.3222..2.325，STL 7f82bc3d1a7c），H01-F09 note 加右前脚 r<2.3 脚顶 233.2/厚 1.685；**H03-F02 重写**为 Ø1.7 自攻底孔（432 射线 r 0.849..0.850，孔1..3 盲深 7.0 刀底 224.5146，孔4 Ø1.7 段 5.0146 + 原版 Ø2.2 残段台阶通孔；placed STL sha 76781c1a566a…）；**H03-F04 删除**（z 231.0/231.3 r 1.4 四孔 12/12 向为料）；**新增 H03-F05 fill**（4×Ø3.6，head.py:15-17）与 **H03-F06 shell_clearance_cut**（DILATE6 0.1，head.py:26）；H03-F03 座 B 的 export_local 坐标由错误的『世界−(26,0,221.1146)』订正为 frames.per_part.H03 的 R/t（x_e=z_w−235.6146，z_e=−x_w+8.1），180 射线 r 11.0768..11.0913，L2 present/through 由红转绿；frames.per_part.H03.export_stl_bbox y ±45.8629 / z max 38.4843、H03 体积 23552.08（trimesh，watertight）；L02-F05 t 8.0→6.3 @y 37.15、export x 18.8→19.65、内面 y 34.0（两孔位）/34.65（两孔位落在头窝里）；catalog 171→172（counterbore 8→7、fill 9→10、shell_clearance_cut 2→3、H03 4→5）。
- fasteners.yaml：F22 pilot Ø1.7（measured）/深 7.0、joins『H03 底孔 Ø1.7 深 7.0』、engagement 6.3（derived，孔4 5.0）、stack 1.7（右前脚 1.685）、channel 4.65、status exported_unverified_on_print；F16 note 0.75→0.775；D-FAST-03 resolution 补一句。L5 抽查（H01,H03，scratch 输出）：F22 self_tap_radial_overlap +0.15 / engagement 6.3 / screw_length_rule 8≤8.4 / 坐面 [1.7,1.7,1.7,1.685] 全绿；engagement_per_hole 仍红：H01 右前脚 Ø2.7 孔在 build #7 网格上被孔底 4 个斜面片并进同一光滑片（max|N·ax| 0.316 > sin6°），detect_cylinders 不认圆柱 → 关联不上（射线 24/24 r 1.35 证明孔在），是检测器对该网格的局限，未改层。
- tolerances.yaml：pla_self_tap F22 unknown→satisfied（6.3，pilot H03-F02）；conflicts.pilot_hole_d 与 fits.pla.m2_pilot_hole_self_tap 加 H03 四脚 + head.py provenance。
- parts.yaml：H03 role/feature_count 4→5/mates_with『接缝已清零…原版×原版基线 67.9』（保留『接缝』+`top_head_shell` 让 l3 _seam_decl 仍走 original_seam_added）/deliberately_added evidence current 0.0。
- assembly.yaml：step 8 的 L_/R_hip_onto_roll_flange 两条 motion 加 note（I5 后预计峰值 0，待 L4 正式扫掠回填，未填 peak）。
- `gate.py --layers 2 --parts H03,H01,N02,L02 --skip-negatives`：本批声明全部 present/diameter/through/depth 绿；剩余红都是 pre-existing 或几何事实（fill 类埋在料里的 WARN、孔周壁 <2.5 的 WARN、H01-F08 通道是开口槽最多 21/24 向、N02/L02 horn_hole 2.2→2.4 retarget 未 build、H01-F05/F06 半座/前板孔的几何红、H01-F14 唇 B 与 H03-F03 同类错误帧『世界−(26,0,221.1146)』未改（09-12 ④ 范围，非本批））。
- **2026-09-13 独立复审（对表 build #7）**：逐条复现上批数据 —— H03-F02 432 射线 r 0.8490..0.8500、孔1..3 刀底 224.5146/孔4 轴心无料；H03-F04 288 点全为料；H03-F03 座 B 5 站 19/36 向 r 11.0768..11.0913、唇 44.3..44.8 r 9.30；N02-F05 三站 36/36 r 7.5160..7.5250、外壁 r 10.0；H01-F08 r_min 2.3222（x_e 15/20 站 31/36）；H01-F09 右前脚 r≤2.28 顶 233.2 / r≥2.35 顶 233.2146、其余三脚 1.70；L02-F05 内面 y 34.0/34.0/34.65/34.65（export x_e 16.5/17.15 互证）；H03 bbox/体积 trimesh 同值；frames R/t 用 placed/ 与导出件 bbox 互证（H03/L02/N02 三件 6 个角点全对上）；l3_static._seam_decl 对 parts.yaml H03 mates_with 仍匹配（key H03_original_seam，mechanical_audit current 0.0）；catalog 172 逐条核对；上批全部为字面替换（无 yaml.dump），HEAD 注释行未丢。复审只改三处：① features.yaml **H01-F14** 唇 B 帧订正（同 H03-F03：axis −z，pos (0.0004,0,−36.45)，span z_e −36.1..−36.8），加 verified_on_export（内圈 Ø18.58..18.60 上半周 17/36 向、外圈 Ø23.17..23.18 仅 6/36 向露出 —— 外圈并进 B 座上半圈的料里，L2 present 由 0/24 变 4/24 仍 WARN，几何事实）；② H01-F08 derived_from 旧行号『head.py:52-53,60』→ head.py:92/101；③ data/README.md 09-12 那句『H03-F04 扩口 / −0.1』加过时说明。L2 复跑（--layers 2 --parts H03,H01,N02,L02 --skip-negatives → scorecard.partial）：H03 3 红（F02/F03 wall WARN、F05 fill WARN）、H01 16、N02 20、L02 15，全部为 pre-existing / 几何事实 / retarget 未 build，本批声明无红。L5 抽查 F22 与上批一致。

## 2026-09-13 —— 审计第一批修复（检查器 + 反例包；数据 yaml 只动 waivers.yaml schema）

依据：`Gate独立审计_2026-09-13.md`（用户逐条核过采纳；口径修正 4 处：禁入体保持 128 边外接只改文本、支撑落点改解析 G-code、L6 目标区间只认 frozen 声明、标定 WARN 只报不判）。每条修复都带"修前红 / 修后绿"的反例。

### 内核 / 入口（core.py、gate.py、waivers.yaml:schema）
- F-核-1：`COVERAGE_ITEMS` 循环在清单为空或条目缺 id 时写 `L<层>/_coverage` FAIL(BLOCK)（以前 `if not want: continue` 静默跳过；assembly.yaml 19 步 0 步有 id，`L4/_coverage` 从未存在）。反例 n_core_coverage_no_id。**assembly.yaml 步 id 待数据侧补。**
- F-核-2：格级 inputs 隐含 `duckstructure/*.py`；L7 `_assign` 把实际读的 placed/*.stl 逐个 `res.inputs`，placed 指纹读不出改 unknown（以前 `continue`）。
- F-核-3：`gate.py --verify [scorecard.json]`：逐格 inputs_hash + source_manifest + git HEAD 与磁盘核对，不一致退出码 3。反例 n_core_verify。当前 out/scorecard.json 已被判作废（33 个输入变了、447 格过期）—— 符合预期，它是 build #6 的。
- F-核-4：豁免 id 必须带判据名（`L5/X:check` 或 `X.5.check`），只到格子的拒绝并记 waiver_notes；`waivers.yaml:schema.bound_input_hashes` 改成 {仓库相对路径: sha256}（旧文档的嵌套键永远匹配不上 manifest）。反例 n_core_waiver_cell_only、n_core_waiver（改用带判据名的 id）。
- F-核-5：新增状态 `RETIRED`（不阻断、不计 INCOMPLETE、单独计数）；l6_motion.py 三处、l7_mass.py 两处硬编码 STALE 改 RETIRED。反例 n_core_retired、n_l6l7_retired_not_stale。
- F-核-6：`--layers/--parts` 子集跑写 `scorecard.partial.{json,md}`（`partial: true`），不写 prev/impact；本轮跑过的层里范围外的格子按 inputs_hash 从 prev 继承（变了 → STALE）。反例 n_core_partial_inherit。实测 `--layers 0 --parts L01`：L01 重算、其余 17 件继承为 STALE（build #7 后 STL 已变）。
- F-核-7：`L0/_negatives` 格无论过不过都写，带 inputs_hash（反例缓存 key 覆盖的文件）。
- F-核-9：反例包结果按 `sha256(negatives/*.py + layers/*.py + core.py + gate.py + tool_access.py + slicing/slice_l1.py + data/*.yaml)` 缓存到 `out/negatives.cache.json`；key 计算 0.03 s；`--no-neg-cache` 强制重跑。
- 反例契约（F-反-1/3/4）：`runner.check_contract` 要求 `expect_severity` + `red`（每条 state==FAIL、severity==期望、measured 非 None，除非 `allow_unknown_red`）或 `assertions`（全 ok；两者都给时都校验）；空壳算失败。`_harness` 新增 `assert_red / assert_green / record / result`、`FakeCtx.mjcf()`；临时件目录改成每进程子目录 `_neg_tmp/p<pid>`（并行互删修复）。
- L6 N4：frozen 或上游 `range_deg` 不是有序 [lo,hi] → `_joints/range_declared_ordered:<关节>` unknown 并跳过该关节（反写时 check_angles 只给两端点 = 没扫）。反例 n_l6_reversed_range。

### 层（另见各层 docstring）
- L2 F-L2-1：`res.covered` 只登记至少发出一条带 measured 判据的特征；geom 无柱体也无 bbox/pos → `<fid>:geometry` unknown；`box_export_local {lo,hi}` 纳入 BBOX_KEYS。`_coverage` 由 171/171 → 160/171（9 条 sweep/cut 无探针 + L05-F09 + L06-F04），假绿转真红，**data 侧补 box_export_local 后恢复**。反例 n_l2_coverage_gap。
- L2 F-L2-2：`<fid>:diameter` 改棱边法（圆柱刻面纵向棱/端面弦上采样点到轴距离），对分段数免疫；比较吸收 1e-5 float32 表示噪声（阈值未动）。64 边 Ø14.85 由 14.834 假红 → 14.85。反例 n_l2_facet_diameter。
- L5 F-L5-3：配合桶 `fit_measured_geometry` 只在 relation_contact PASS 时 PASS；量了没过 → FAIL(BLOCK, measured=关系 id)。反例 n_l5_fit_measured_only_on_pass。
- L5 F-L5-1：l5_screwhead 跳过 kind=pilot_hole 接收孔（与 `_group_holes` 分类一致），detail 记跳过数；F24 的 −19.49 假红消失。反例 n_l5_head_on_pilot_hole。
- L5 F-L5-2：头侧只从 `fasteners.yaml:tool_access[].seats[].outward_export_local` 取（feature_id+map_index+hole_index 对孔）；取不到 → `head_side`/`seat_flatness` unknown，不取较差端。l5_function 坐面改按螺丝组穿件孔逐组判（subject=螺丝组），`seat_flatness`=头侧平面度+每向有料，新增 `seat_stack_spread`=叠厚极差；删除按直径扫全件的 `_screw_holes`。**没有 tool_access 的 28 组现在全 unknown（等坐面探针合并）。** 反例 n_l5_fg15（+远端台阶/未声明两例）。
- L1 F-L1-1/2/3：`_calibration/proven_parts` 范围内 BLOCK 判据 n_eval=0 或记录缺失 → unknown；新增 `warn_checks_over_limit` FAIL(WARN) 只报不判；calibrate_l1 把 PASS&evidence_n=0 计 unknown。`support_in_no_support_zone` 的"下界超"判红前要求本件 `support_model_brackets_slicer` PASS，否则 unknown（过渡保护；真实 G-code 支撑落点解析排第二批）。`l1_calibration.yaml` 已重跑（见文件头日期）：三条 BLOCK 判据 n_eval=0 → `L1/_calibration` 会是 unknown 红，直到标定件带切片记录。反例 n_l1_calibration_record（+3 例）、n_l1_support_in_bore/_box_zone（用层自报区间中点的切片桩通过过渡保护；桩不证明模型对，只让判据可检验）。
- L7 F-L7-2：`inertia_tensor_parsed` 改 n_I == body 数（以前"至少一个"）。反例 n_l7_inertia_tensor_all。
- 反例包（F-反-2/3）：六个只测辅助函数的改为经层 run；新增 n_l0_units_meters（N7；N8 Y-up 记为已知缺口）、n_l3_bearing_section（N2 + N1 空刀）、n_l6_reversed_range（N4）、n_l7_mass_torque（舵机 ×20 / 电池 −x 200 mm / forcerange ±0.7）。n_l1_support_in_bore 圆孔改方孔切（l1 支撑模型在 96 边圆孔上 shapely 420 s → 3 s，**l1 性能待查**）。README 事故表加文件名列；N5（螺丝加长 2 mm）、N6（删一个件）仍无专门反例。
- 未做（第二批）：F-L4-1 present 推导、F-L3-2/3、F-数-1 冻结项真核、F-L7-3/4/5/6、F-L6-1/2/3（目标区间只认 `frozen.yaml:joint_axes[].target_range_deg`）、F-L5-4/5、L3/L7 泛化、F-L3-1 文本 + keepouts.yaml facet_noise_mm3、G-code 支撑落点。

## 2026-09-13 —— 主线程：切片器常驻、build #7 基线、N03 数据、坐面探针

- 切片器：PrusaSlicer 2.9.6 dmg 重新下载（github prusa3d release，sha256 94fd7b8a…）装到 `/Applications/PrusaSlicer.app`；`slicing/slice_l1.py` 默认 `--exe` 指过去，`slice_run.slicer` 记出处。18 件按 build #7 重切（`docs/build_logs/slice_2026-09-13_build7.log`）贴进 `printability.yaml:slice_run`：rc 全 0，`layers_single_bead` 18 件全 0（L03 101 层只有外圈但 0 层单 bead，与 09-12 解析器修正一致）。
- N03（调查 I2 + 真切片附录，`docs/probes_2026-09-12/geometry_investigation/I2_N03_print_orientation.md`）：18 个朝向没有零侵入的，朝向不动；features.yaml N03-F05 `bore`→`clearance_cut`（让刀腔，无配合面）；N03-F07 `axial_span` 由 40 mm 刀长（把舵机腔圈进禁撑区）改为背板料厚并集 z_e 29.75..35.65（两排各 3.0，实测 y=7.5 排 29.75..32.74、y=−22.5 排 32.65..35.65），catalog bore 5→4 / clearance_cut 5→6。真 G-code 复核：`buildplate_only` 下 F02/F04/F05/F06 归零，F07 四背孔仍有支撑柱穿到床面（真几何，需逐特征 removable 豁免 + 装前 Ø2.4 通孔）。
- Gate 全跑基线（build #7 + 09-13 数据对表，审计第一批落地前的层代码）：979 s，85 PASS / 351 FAIL / 7 NOT_RUN / 1 RETIRED，BLOCKED（`docs/build_logs/gate_2026-09-13_build7_baseline.log`）。对 09-10：L3 38→26 红、L4 70→50、L1 切片类判据全绿；两个反例失败是审计批次改到一半的契约/标定反例，非件的问题。
- 真切片支撑落点分析（`docs/probes_2026-09-12/geometry_investigation/真切片支撑落点_2026-09-13.md`，分析表非判据）：auto 支撑下 H01 两轴承半座 + Ø22.78 座孔、L01 6704 座、N02 Ø15.05 通道、N03 Ø16.2 导向孔里都有真支撑；`buildplate_only` 只救 T01/L05/N03，H01/L01 轴承座救不了（待用户定：朝向/可拆桥顶/装前铰刀）。建议禁撑区分 `forbid` / `removable(post_process)` 两级。
- 坐面探针（`tools/gate/probe_seats.py --groups <一组>`，串行 24 组，`docs/build_logs/probe_seats_2026-09-13.log`，逐组输出 `out/probe/<gid>.{yaml,log,stdout}`）→ 合并进 fasteners.yaml 见下一条。
- 坐面探针合并（09-13 深夜）：24 组四轮跑完（探针修 3 个 bug：T_RANGE 15→45 粗细扫、`contains` 分块 200 防 5 GB 被杀、fine ts 先 round 再 unique），并入 fasteners.yaml → 25 组 `tool_access`、143 坐面、每组 `tool_access_probe_2026-09-13` 说明；F09 上排 hole0 两侧坐面按 0.05 步复量手工写入（`probe_note`）。F20 无坐面（曲壳，落差 0.19/0.63）、F05/F07 上排为垫管座只写说明。L4+L5 子集跑（F12 合并前）：`tool_reference` unknown 27→7；F06/F22/F03 转成有数的红（工位或头侧待核，见 docs/交接_2026-09-13.md §2c）。主线程数据修改到此为止。

## 2026-09-13 —— 审计第二批（主线程 + 子 agent A/L6/L7 + L1 agent；检查器 + 反例 + 数据字段）

依据 `docs/gate/Gate独立审计_2026-09-13.md` 与用户 09-13 第二批任务书（A 审计条目、B L5 对齐、C L1 真切片落点、D L4 数据复核、E 缓存 key）。每条修复都带"修前红 / 修后绿"反例；阈值一个没改、判据一条没删。全量跑与归档见本节末。

### L4 / 装配数据（F-L4-1、D14、D15；主线程）
- F-L4-1：`l4_assembly.py` 新增 `intro_table / derive_present / installed_upto / judge_present`：每个动作与每个 `tool_states` 工位判 `present_covers_derived`（BLOCK：present ⊇ movers ∪ 早于本位置、所属子总成 ∈ closure 的实体，按左/右/中央侧过滤）与 `present_only_installed`（WARN：present 里出现按 seq 还没装 / 没有任何一步装的实体；同步稍后动作的只记 detail）。子总成闭包读 `assembly.yaml:subassemblies`，归属读每步 `subassembly_of`（motions 可逐动作覆盖），顺序读 `seq`。缺声明 → unknown。反例 `n_l4_present_derived.py`（漏写已装件红 measured=1 / 补上绿 / 顺序矛盾 WARN / 缺子总成 unknown）。
- `_Names._lookup` 轴承按 `components.yaml` 型号/三元组解析外径（`_bearing_od`），不再写死 6700→15/6704→27；`6702×2`、`22×16×4` 现在能落到 placed 实体（以前 unresolved）。型号代码只认 model 开头（D-COMP-01 那句"错标成 6704ZZ"不登记）。
- `assembly.yaml`：每步加 `id`（step01…19，L4/_coverage 用）、`seq`（物理顺序 1,2,3,4,5,7,9,10,6,8,11,12,13,18,14,15,16,17,19）、`subassembly_of`；顶层 `subassemblies`（ankle_arm/ankle/upper_leg/leg/trunk/body/head/neck/neck_head/robot）；`already_installed` 按 seq 重算；`tool_states.*` 加 `after_step`/`after_motion`/`side`/`subassembly_of`。文本级替换，注释未动。
- D15 F22：`tool_states.head_shells_on` 去掉 `orig_top_head_shell`（探针：4/4 头只被顶壳挡 22–39 mm；去掉后刀路 0 mm³、4/4 可见）→ 物理顺序先拧 F22 再盖顶壳，`after_motion: H03_bottom_shell`。
- D15 F06：步 8 重排 —— 新增 `L/R_hip_onto_pitch_flange`（L02 在腿台面贴俯仰法兰，subassembly leg，沿俯仰轴抽出），`L/R_hip_onto_roll_flange` movers 改为 L02+整条腿、len 60→100（60 mm 证不出脱离），删 `*_upper_sub_onto_hip`；`hippitch_flange_*` 工位改腿台面（12/12 可见、0 mm³），`hiproll_flange_*` present 加腿（F04 带腿复算 3.3e-6 mm³、12/12 可见）。步 8 十条动作本机扫掠全绿。
- D15 F03：结论"真拧不到（直杆）"—— 内排两颗从坐面沿 −x 13.86 mm 撞 T01 电池仓前壁（厚 3.5，Ø4 批杆 44 mm³）；不改数据不改件，三条处置路线见 `docs/gate/L4数据复核_2026-09-13.md` §3。
- D14 复核：36 动作 / 24 工位 present 全部 ⊇ 推导集合；`orig_face_part`/`orig_jaw` 没有任何装配步（步 17 present 里出现 → WARN）；步 6 右"左腿已完整"的假设在 step 粒度下表达不了（WARN 记录）。全文 `docs/gate/L4数据复核_2026-09-13.md`。

### L3 / 内核 / 冻结项（主线程）
- F-L3-2：`l3_static._ko_geoms` KO01 不再 `import conn_cut`：走廊 (c) 由 `keepouts.yaml:KO01.geom_verbatim(c)` 的截面与 y 区间 + `dims.socket_center_mm` 建盒，窗口 (a) 由 `dims.conn_x_mm/conn_z_mm` 在 y=±y0 面上按 `window_face_grid_mm`（0.5）取点、离面 `window_face_probe_offset_mm`（0.01），点落在打印件实体内 = 面上有料 → `keepout_KO01_window_face` FAIL(BLOCK)（口袋壁隔 0.3 盖住窗口边缘环带不算）；舵机局部→世界只借 `duckstructure.lib.sfw`（落位）。CAD `conn_cut` 只作对照 `keepout_KO01_vs_cad_cutter` INFO（独立−CAD / CAD−独立 体积）。dims/网格缺 → `keepout_dims_declared` unknown 且不留 0 对的 PASS。真数据：`keepout_intersection` 5.9e-05 与基线同、窗口面 14×234 点全空。反例 n_l3_ko01_independent（声明走廊加长到 20、料放 y=25 → CAD 切刀看不见、独立实体 64 mm³ 红；贴面的口袋壁红 / 隔 0.3 绿；dims 缺 unknown）。
- F-L3-3：`components.yaml:components[].envelope_solid`（ring: bore/od/t；box: size_mm + overall_mm + exact；none + why；box 三边按 overall_mm 与 OBB 三轴长度配对，两个 overall 值差 <0.5 分不出轴 → unknown 不猜 —— 舵机 T19.9/W20 差 0.1 配错轴曾在全量跑里造出 H02 垫柱 × 薄段 1.36 mm³ 假红，用 overall 25.7 = 厚段 22.8 + 法兰 2.9 分辨后归零）→ `_envelope_solid` 独立建实体，落位只借 placed 的 OBB（`trimesh.bounds.oriented_bounds`），尺寸不抄 CAD/placed；`placed_covers_envelope`（顶点法：OBB 三边 / 环的顶点内外径 vs 声明 ±`envelope_vs_placed_mm`，box 非 exact 时 ⊇）+ `part_vs_component_envelope` / 件格 `component_envelope_<id>`（BLOCK）；原 `part_vs_component_solid` criterion 写明"实体 = build 导出的 placed 占位体，与 CAD 同源"。7 条元件补 `envelope_solid`（servo 34×20×19.9 薄段 ⊇、4 种轴承环、电池 72×25×18 exact、original_prints none）。真数据全部 PASS（servo 225 对、6704 100 对…峰值 0）。反例 n_l3_component_envelope（占位体挖 3×3×3 缺口 + 载体筋坐在缺口里：同源判据 0 绿、独立实体 8 mm³ 红；不声明 → unknown）。
- §2.6（L3）：元件认领改读 `components.yaml:claim`（与 L7 同一份：mjcf_servo_geom 名单来自 MJCF body.servos `servo__<body>_<drives>`、envelope_ring/box、placed_prefix），不按 category；KO06 `servo_frame_of_joint`、KO07 `axis_joint`、KO11 `probe_zone_z_mm`（盒 x/y 外沿取 placed 包围盒）、KO12 `exempt[]`（part+segment+why）、KO15 `fov_deg/apex_world_mm/optical_axis_world` 全部读数据，缺 → 一条 unknown（`*_declared`）；`_DEFERRED` 表删除 → `keepouts.yaml:KOxx.checked_in_layer`（KO02/03/04→4，KO10/13→6）+ `covered_by_subjects`（KO02: 9 步含舵机滑入；KO03: step01–04；KO04: disasm03–05），`core.obligations` 按它派义务、`core.build_scorecard` 汇总成 `L{层}/KOxx`（`covered_by_cells`：缺格 NOT_RUN、状态取最差、evidence 求和）；件格 `keepout_*` / `component_*` 的 evidence_n 改为查过的对数（以前是超阈值对数）。反例 n_l3_keepout_data_driven、n_core_keepout_layer_routing。
- F-数-1：`l3_static._frozen_checks`：`frozen.yaml:battery_bay_26.measured_on{bay: T01, door: B01}` → 仓中心射线量 BX1/BAY_FW/BZ0/BAT_Y/RAIL_Y、导轨带射线量 RAIL_Z0/BZ1、SHELL_BOSS 四柱有料、门板 DOOR_T/BX0/DOOR_YLO、底脚 FOOT_Z/FOOT_X[1]/FOOT_Y（FOOT_X[0] 与门板连成一体不单独核）、SNAP 卡珠面与导轨面差 ≤0.1 射线分不出 → unknown；`h01_sbc_posts.measured_on: H01` + `post_axis_world` → 柱外沿内 3 mm 截面闭环：count 4 / hole_pitch [58.08, 23.02] / post_d 5.0 / pilot_d 1.7 全 PASS；`imu_pose.measured_on: T01` + `mount_hole_d_mm 1.7` → ±y 射线扇找到孔心 y ±8.0 均值 0 PASS、生成 MJCF `<site imu>` 与 frozen pos_local_m 偏差 0 PASS、delta 算术 PASS；9 个非几何项（BAT_CLR / 死常量 / SERVO_SHIFT / P / 3 个函数体）登记 INFO"由 build 的 verify_frozen 锁定"。阈值 `tolerances.yaml:frozen_dimension_mm ±0.1（src: assumed）`、`envelope_vs_placed_mm ±0.1（src: assumed）` 新增。反例 n_l3_frozen_items（仓半宽错 0.5 / 柱距错 1 / IMU 孔偏 1 红，对照 14 条绿，measured_on 缺 unknown）。
- L3 子集跑（`--layers 3 --skip-negatives`）：92 条/37 红 → 135 条/40 红；新增红 = SNAP unknown（射线分不出）+ 新 unknown 只在无 claim 的 14 类未建模元件（原本就是 world_placement unknown）；`servo_s288` 14/14、轴承 9/9、电池 1/1 认领；`original_prints instance_count 3≠4` 与基线同。

### L1 支撑落点（L1 agent + 主线程收尾）
- C-10：`slicing/slice_l1.py` 解析 `;TYPE:Support material` / `Support material interface` 挤出段（G1 带 E 且 XY 位移；类型跟层），按 `criteria.support_landing_sample_step_mm`（0.2，含端点）采样，z = 层顶 `;Z:`，经 `orient_transform`（align_vectors M + shift）逆变换回 export_local，对 `l1_printable.no_support_zone_set`（层与脚本共用，`zones_sha256`）逐区数点 → `slice_run.parts.<件>.support_landing{per_zone, hits_forbid_samples, hits_removable_samples, support_path_mm…}`。`l1_printable.support_in_no_support_zone`（forbid 级，BLOCK）/ 新 `support_in_removable_zone`（WARN + 逐特征后处理文本）/ `support_model_zone_crosscheck`（模型上下界 vs 真落点，INFO）；没有记录 / source_sha256 或 zones_sha256 对不上 → unknown。
- C-11：`printability.yaml:no_support_zones.tiers`（forbid: bearing_bore/journal/spot_face + 走廊 `mating_shells_export_local` 壳层；removable: 其余 7 类，含 slide_corridor 口袋盒，各带 post_process），`from_features_yaml_kinds` 必须 = 两级并集；`per_feature_overrides`：N03-F07（removable，装前 Ø2.4 通四背孔）、N03-F02（Ø16.2 导向孔顶清孔）、**H01-F06**（主线程 09-13：留 removable —— head.py:44 实测前立板承压圈 r 6.8，这个 Ø22.78 孔不是 B 盘/轴承入口，只是拆 N04 的 Ø6 顶杆通道，无配合面；`features.yaml:H01-F06.purpose` 同步订正并留 purpose_note；手算表 §3.1 把它归 bearing_bore 是按旧 purpose）。
- C-12：`printability.yaml:parts.N03.slicer_args: [--support-material-buildplate-only]` → argv 追加，记入 `slicer_argv_extra`。
- C-13：`features.yaml:L01-F13 / L03-F05` 加 `geom.mating_shells_export_local[]`（配合面 0.5 mm 壳层，forbid），口袋盒本体降 removable。
- 主线程收尾：`slice_l1.py` 的 WORLD_DOWN / PLACED_NAME 表与 `if pid == "N02"` 分支全部搬进 `printability.yaml:parts.<件>`（`down_world` / `fill_density_percent` / `perimeter_generator` / `slicer_args`；缺 → 退出码 4 不给缺省；placed 名取 parts.yaml `placed_instances[0]`）；`--parts` 子集重切沿用其余件记录；`no_support_zone_set` 的 sha 只带**本件**特征的 override（原来带全局 override，改一条 override 会让 18 件记录全部作废）。18 件全部重切（`slice_run.date 2026-09-13`，18/18 source_sha256 与 zones_sha256 对上）：forbid 落点只剩 H01 145.4k（F05 两半座）、L01 46.8k（F10 + F13 壳层）、L03 8.3k（F05 壳层 + F06）、H02 2.4k（F07 锪平），其余 removable WARN。逐件对照手算表 `docs/gate/支撑落点对照_2026-09-13.md`。反例 n_l1_support_landing_tiers（forbid 红 / removable WARN 带后处理 / 声明变了没重切 unknown）、n_l1_support_in_bore / _box_zone 改为落点桩（`_harness.landing_stub`）。
- E-16：`gate.py` 反例缓存 key = 静态文件全集（`tools/gate/**/*.py`、`duckstructure/*.py`、`tools/cad/*.py`、`data/*.yaml`、`slicing/*.yaml`、`l1_calibration.yaml`、标定网格）+ 运行时 `sys.modules` 仓库内模块清单 + trimesh/manifold3d/numpy/shapely/Python 版本；命中时 `L0/_negatives` 照写并标 `cached`。反例 n_core_neg_cache_key。

### L5（agent A）：L5 头判据起点改坐面 + 阈值来源 + 盲深字段（检查器 + 反例；数据只补 `tolerances.yaml` 阈值 `src` 与 `data/README.md` 一句）

依据：主线程 09-13 对齐问题（25 组 `tool_access` 143 坐面并入后，14 组 `head_seat` unknown "孔落在导出包围盒外"、F01/F06 "声明头侧与实测孔轴不共线"、F02/F03/F10 "6/12 孔头侧未声明或左右冲突"）。真因：`l5_screwhead` / `l5_function` 第 2 段拿 `feature_hole_map` 的孔点当射线起点，而它是**刀心**（切刀中点，常离件 15–19 mm 甚至在件外）；`declared_head_sides` 的键没有 `instance`；同一颗螺丝的沉孔条目被当成另外 6 颗。每条修复都带"修前红 / 修后绿"的反例（先写反例、在旧代码上跑出失败，再改层）。

### 层（l5_screwhead.py / l5_function.py）
- F-L5-2 第二半（起点）：头判据按 `tool_access[].seats[]` **逐坐面**判，起点 = `point_export_local`、头侧 = `outward_export_local`；`feature_hole_map` 孔点只当**轴线锚**（坐面点到孔轴横向偏差 > 1e-6 → `head_seat` unknown，与 `tool_access.py` 同规则）。孔轴优先取特征声明轴（`geom.instances[k]` 按 `feature_hole_map[].instance` 取，`hole_geom()`），实测轴起点改到孔腔里（坐面点沿 −outward 进 0.5×孔径），`hole_axis_declared_vs_measured` 保留为旁证。射线只认坐面外侧 ≤ 头高（`frozen.yaml:P_dict.m<N>_head.h`）内的第一张进料面（`_seat_end`，按面法向分进/出），头高区外的杯壁/另一堵墙不算坐面；该孔叠厚由同一批射线（足印内圈）的"进料面→下一张出料面"给，删掉 `_first_run`。足印环带在坐面头侧 1e-3 mm 处全是料 → 坐面点在料里 → unknown（不报成"头悬空"）。逐孔判据名改 `head_seat_flatness:hole<map_index>.<hole_index>`。反例 n_l5_seat_origin_cutter_mid（刀心在件外 20 mm：修前 `head_seat` unknown → 修后 `head_seat_exists`/`head_seat_flatness`/`seat_flatness` PASS；坐面挖 0.5 台阶 → `head_seat_flatness` FAIL(BLOCK) 0.5；坐面横向偏 0.3 → unknown）、n_l5_fg15 新增 E（坐面点写在料里 → `head_seat` unknown）。
- F-L5-2 第二半（键）：`declared_head_sides` → {(feature_id, **instance**, map_index, hole_index): outward}，冲突只在同一 instance 同一孔方向不一致时成立；新 `seat_table()` 把左右实例（mirror_y，同一份 STL、export_local 同坐标）同 outward 的坐面合成一行，几何量一次、detail 记"坐面覆盖 instance [...]"，evidence 不重复计；某 instance 缺坐面 = 那颗螺丝未声明。头侧判据只覆盖 `head_locator_map_indices` 指向的 map 项（沉孔/铣面条目不进"未声明"，也不作为叠厚/坐面测量孔）；locators 缺/重复/越界 → `head_side` unknown；任一头侧孔取不到坐面 → 该组 `head_seat`/`head_side` unknown，不用部分孔冒充全组；没有 `tool_access` 的组保持 unknown。`l5_function` 第 2 段同口径（`_group_holes` 带实例几何、rows 限 locator 条目、坐面点必须在孔轴上）。反例 n_l5_seat_instance_key（L/R 同向 → 绿且探孔数 2 不是 4；同 instance 反向 → `head_side` unknown；右实例缺一坐面 → unknown 点名 instance）。
- F-L5-4（阈值来源）：新 `threshold_meta()/src_label()/AssumedThresholds`：`fits.*.target_range_*`、`feature_check_tolerances.{seat_flatness_spread_mm,horn_clock_deg,horn_concentricity_mm,hole_depth_mm}` 读取时把 "阈值 src=…" 写进 criterion；节点无 `src` → 引用它的判据 unknown（"阈值没有来源"：`fit_value` / `relation_contact` / `seat_flatness` / `seat_stack_spread` / `horn*:clock_index` / `pattern_radius` / `head_seat_flatness` / `stack_from_seat_crosscheck` / `counterbore_diameter`）；`src=assumed` → 照判 + 每 (subject, 键) 一条 `threshold_assumed:<键>` FAIL(WARN)，measured=阈值本身（fits 桶挂在桶格上，feature_check 阈值挂 `L5/_thresholds`；两个模块各发自己的一条，同键会出现两条）。**未纳入**：`engagement_by_joint_type.*.range_mm`、`screw_length_rule.tip_margin_mm`、`hole_diameter_mm`（L5 只当关联容差）。`fit_on_placeholder` 保持 WARN。反例 n_l5_threshold_src（assumed → WARN 且 measured 非空；删 src → `fit_value`/`head_seat_flatness` unknown；measured → 无 WARN）。
- F-L5-5（盲深）：`_blind_depth` 先读 `fasteners.yaml:<组>.pilot.pilot_depth_mm`（`core.num`），detail 写 `pilot.pilot_depth_mm（src=…）`；取不到再退回 joins/location 正则；两者都有且不一致 → `screw_length_rule` unknown 并把两个数写进 detail。反例 n_l5_pilot_depth_priority（pilot 3.0 → 8 > 4.0+3.0−0.3 红；joins 4.5 → 绿；3.0 vs 4.5 → unknown；4.5/4.5 → 绿）。
- 子集跑（`--parts`）范围外的穿件孔不再报 "N 个声明孔一个都没关联到 []" 的 `engagement_per_hole` unknown（F19/F20/F21/F24 在 L01,L02,H01,H03 子集里）；全被跳过就不发。

### 数据
- `tolerances.yaml`：32 处阈值节点补 `src: "assumed"` + `src_note: "2026-09-13 补 src：区间由本文件提出，无试件/手册来源"`（29 个 `fits.*.target_range_mm/deg`；`feature_check_tolerances.hole_depth_mm/horn_concentricity_mm/horn_clock_deg` 注明"由 tools/gate/README.md 第 2 节提出、本文件转录"；`fits.battery_bay` 注明冻结采用值 BAT_CLR）。**不补**：`fits.axis.concentricity.target_range_mm`（区间引用 `datum_vs_mjcf_mm`，自己没 src → 引用它的判据 unknown）、`fits.tpu.process`（区间 null）。已有 src 的（`seat_flatness_spread_mm`）不动。这是如实记录，不是编数。
- `data/README.md` 字段约定加一句：`hole_positions_mm` 是刀心，L2 用 `axial_span_mm` 兜住，其它层不许当坐面/起点，头/坐面一律用 `tool_access[].seats[]`。

### 子集跑 `--layers 5 --parts L01,L02,H01,H03 --skip-negatives`（改前 343 条/221 红 → 改后 383 条/247 红；多出的红基本是 31 条 `threshold_assumed` WARN）
- F02：`head_side` unknown（"6/12 未声明"）消失；`head_seat_exists 0/6` / `head_seat_flatness 0.0` / `stack 5.35` 不变，detail 记覆盖 yaw2roll + yaw2roll_R。
- F03：`head_seat`（包围盒外）/`head_side` 两条 unknown → 有数：`head_seat_exists 0/2` PASS、`head_seat_flatness 0.0` PASS、`counterbore_diameter 4.4` PASS；**新红** `stack_from_seat_crosscheck` FAIL 2.99（记录 3.29，差 0.30；独立射线复核 L01_yaw2roll.stl (8±1.366, y, 5) 沿 +y：进料 −13.24 → 出料 −10.25 = 2.99，与 L01-F09 purpose "余 3.0" 一致；3.29 来自 09-09 final_measurements，疑为旧 build）。数据未改。
- F06：`head_side`（"与实测孔轴不共线"）unknown → `head_seat_exists 0/6` / `head_seat_flatness 0.0` / `stack 7.5` PASS（声明轴 −x 按 `geom.instances[1]` 取）。
- F22：全部不变（`seat_flatness 0.0`、`head_seat_exists 0/4`、`stack 1.685`、`engagement_per_hole` 仍红）。
- F01 未在子集内；T01 探针：seat0 (−16.5,25.5,27.65) 头侧 +1e-3 环带 0/72 在料、内侧 72/72 在料，−z 穿越 27.65 进 / 24.65 出 → 坐面点在料面上、叠厚 3.0，坐面数据可用（点与刀心重合是巧合，不是复制错误）。
- 反例：10 个 `n_l5_*` 全过（含 4 个新建 + fg15 加 E），单个 0.75–4.55 s。

### L6（agent L6）：F-L6-1 / F-L6-2 / F-L6-3（审计 `docs/gate/Gate独立审计_2026-09-13.md` §L6）

### `frozen.yaml`（只动 `joint_axes[]`，文本级插入，注释未丢；`yaml.safe_load` 通过）
- **15 条 `joint_axes[]` 各新增 `target_range_deg`**（`frozen.yaml:26-29` 起，每条紧跟 `range_deg` 之后）：
  14 条 MJCF joint 写 `{v: [lo, hi], src: "<上游 robot_walk.xml 路径:行 <joint name range=弧度原值>（弧度→度，保留 1 位小数）", src_note}`，
  值全部 = 上游 `upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml` 的 `<joint range>`
  （**没有**用生成的 `sim/duck_s288/robot_walk_s288.xml`）；现在没有人为收窄，所以没有 `date`/`reason`。
  `mouth`（`exists_as_mjcf_joint: false`）写 `v: null` + `unknown_class: not_applicable` + `unknown_reason`。
- `range_deg` 与其余字段一个没动；`counts.joint_axes` 仍是 15。

### `tools/gate/layers/l6_motion.py`
- **F-L6-3 目标区间**
  - `l6_motion.py:82-83` 新常量 `_TARGET_KEY` / `_TGT_TOL=0.1°`；`:359` `_target_range()`：只读 `frozen.yaml:joint_axes[].target_range_deg`，
    缺 / 不是 [lo,hi] / 非数 / lo≥hi / 超出上游扫描域 / 人为收窄（与上游差 >0.1°）无 `date`+`reason` → 不可用；`:507,533` 在关节清单循环里分成 `tgt` / `tgt_bad`。
  - `:649` `in_target(pose)`（非零关节全在各自目标区间内；含不可用关节 → None）、`:687` `sev_of()`（区间内有一条 → BLOCK，全在区间外 → WARN，分不出 → None=unknown）、`:695` `ws_note()`（"工作空间问题（控制端限位），不是件的问题"）。
  - `:448` `crit` 改写为新口径（含"目标区间只认 frozen.yaml 声明"）；`:773-806` `single_axis_sweep`：目标区间不可用的关节不扫、判 unknown（detail 写缺 target_range_deg），其余按 `sev_of` 定 BLOCK/WARN；
    `:834-845` `pair_combination` 父关节不在清单 / 任一关节目标区间不可用 → unknown；`:888` 严重级按两轴都在区间内判。
  - `:949` `min_clearance`、`:982-988` `collision_buckets`、`:1482-1486` `motion_collision`、`:1531-1545` `keepout_motion`、`:1129` `combo_collision_inside_free_range` 同口径（含 None → unknown）。
  - `:1014-1120` **`collision_free_range` 改判**：解析生成的 `sim/duck_s288/robot_walk_s288.xml`（`:385` `_mjcf_joint_ranges_deg`），每条 `<joint range>` ⊆ 本轮无碰撞区间否则 FAIL(BLOCK)（`:1106`）；
    新增 `gen_range_covers_target`（`:1113`）：⊇ `target_range_deg` 否则 FAIL(WARN)。生成文件缺 / 解析不了 / 有关节未扫 → 两条都 unknown。"需要收窄"只写进 detail，不再是 BLOCK 理由。
  - `:84` `_RANGES_OUT` 常量取代写死的导出路径（反例改指临时文件，不再覆盖真 `out/mjcf_ranges.json`）；导出 JSON 多了 `target_range_deg` / `unscanned_no_target_range` 两键。
- **F-L6-1 网格**：`:80` `_COMBO_STEP_DEG = 5.0` 取代 `_COARSE_DEG = 10`（原 `max(10°, 量程/12)`）；`:847-848` 两轴统一 5° `check_angles` 网格；
  `:831` criterion 写明步长；detail 记 `5.0° 统一网格 A×B 个姿态（+ 峰值粗格附近 2.5° 加密 N 个姿态）`。真场景逐对实测：组合扫描 44 s → 351 s（`head_pitch+head_yaw` 5.5 s → 86 s 最重）。
- **F-L6-2 豁免**：`:219` `mating_pairs` → `zero_gap_pairs`（只是几何事实，不再是豁免依据）；`:398` `_contact_pairs()` 从 `relations.yaml:*.contact.parties` 解析件对
  （`placed` 认单只实体、`part` 认该件号全部实例，经 `parts.yaml` 桥接的 `stem2pid`）；`:701` `stem2pid` 前移到零位姿基线之前；
  `:732-760` 新判据 `_zero/declared_contact_pairs`（解析不到的声明 FAIL(WARN)）与 `_zero/undeclared_zero_gap_pair`（零位间隙 < 阈值但没声明的对，FAIL(WARN) 列出，不阻断）；
  `evaluate()` 的 `mating` 参数改为只收声明对（`:185`）。
- `:1572` `res.evidence` 新增 `records_in_target_range / records_outside_target_range / records_target_unknown / joints_without_target_range / combo_grid_step_deg / declared_contact_pairs / undeclared_zero_gap_pairs`。

### `tools/gate/negatives/`
- 新增 `_l6_scene.py`（不以 n_ 开头，runner 不收）：替身 placed/ + 真运动树 + 可注入合成 evaluate / 只留几只实体 / 只扫几条关节 / 临时生成 MJCF / 猴子补丁 `PLACED _Scene _GEN_MJCF _RANGES_OUT` 并还原。
- 新增 5 个反例（都先在改代码前跑出 passed=False，再改到 True）：`n_l6_combo_pocket.py`（F-L6-1）、`n_l6_target_range_severity.py` / `n_l6_target_range_declared.py` / `n_l6_gen_range_vs_free.py`（F-L6-3）、`n_l6_undeclared_zero_gap.py`（F-L6-2，真几何真布尔）。
- 既有 `n_l6_reversed_range` / `n_l6_scene_inventory` / `n_l6l7_retired_not_stale` 改完仍过；8 个 L6 反例都 < 5 s、`runner.check_contract` 全 None。
- 主线程核对 L6 子集跑（`--layers 6 --skip-negatives`，460 s）：与 out/scorecard.json 相比 `collision_free_range` BLOCK 1→0（生成 MJCF 6 条收窄 range 全部 ⊆ 本轮无碰撞区间）、新增 `gen_range_covers_target` WARN 1（训练域被收窄）、`undeclared_zero_gap_pair` WARN 1；其余 BLOCK 数不变 —— **因为 target_range_deg 现在全部 = 上游区间**，扫描域内的碰撞都在目标区间内。要把"工作空间问题"转 WARN，需要人在 `frozen.yaml:joint_axes[].target_range_deg` 里带 date+reason 收窄，Gate 不代做。

### L7（agent L7）：F-L7-3/4/5/6 + §2.6 L7 泛化（检查器 + 反例 + 数据只加字段/删 yaml 自述要删的键）

依据 `docs/gate/Gate独立审计_2026-09-13.md` §"L7 质量与力"与 §2.6。每条先写反例、在旧层上跑出失败（scratchpad `neg_l7_before/`：反写区间 `static_torque` PASS evidence_n=2；删电池质量 `com_in_support_home` 仍 PASS 13.353；无 `component_qty`/`component_body`/`torque_source_consistent` 判据；6 对声明被 `== 5` 判红），再改层，再跑到全过。

### 层（tools/gate/layers/l7_mass.py）
- F-L7-3 `static_torque`（l7_mass.py:1156-1245）：`range_deg` lo ≥ hi 或 `check_angles` 评估点数 < 3 → 该关节 `static_torque` unknown（以前反写只给两端点、evidence_n=2 也 PASS）；criterion 写明"评估点数 ≥ 3（check_angles 2.5° 零度对齐网格 + 两端，lo<hi 且宽度 ≥ 2.5° 时必然 ≥ 3；== 2 就是退化）"。新增 `joint:<j>:static_torque_home`（l7_mass.py:1224-1245）：全部关节同时置 frozen home_rad，同阈值（额定 × margin，margin null 时按必要条件 1）单独判 PASS/FAIL，measured=占额定比。`_head_budget` 同样跳过区间无效的关节并判 unknown（l7_mass.py:1712-1750）。反例 n_l7_static_torque_home_and_range（反写 left_knee → unknown；head_pitch home 取反 + 头偏航舵机实体沿 +y 平移 1.5 m + 舵机 ×4 → head_yaw 扫描 0 绿、home 1.2155 红；对照 14+14 全绿）。
- F-L7-4 质量清单缺口联动：`_mass_items` 第 5 个返回值 `gaps`（l7_mass.py:573-833；缺口 = qty>0 无质量 / 无 claim / 认领数≠qty / 定不出 body / 有质量认领不到实体 / qty 非整数 / 密度无来源）+ run 里未归属的打印件（l7_mass.py:896）。gaps 非空 → `static_torque` / `static_torque_home`（l7_mass.py:1214-1233）、`com_in_support_zero|home`（l7_mass.py:1623-1628）、`head_mass_budget`（l7_mass.py:1759-1763）一律 unknown，detail = "质量清单不全 → unknown，不给下界当绿。仅含已知质量的**下界**：…" + `_gap_text(gaps)`（l7_mass.py:1271）；`worst_joint_torque_ratio` / `total_mass_vs_mjcf` / 单脚余量 INFO 注明是下界。真数据现有 32 项缺口 → 这些判据现在全部 unknown（**这是要的结果**，见下"子集跑"）。反例 n_l7_mass_gap_unknown。
- F-L7-5 额定扭矩唯一来源：`_rated_torque_Nm`（l7_mass.py:137-149）只读 `tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm`（yaml 自述 authoritative），删掉镜像键优先读取与 docs/ 散文 grep 退回路径（连带删掉只在退回路径上才会发的 `rated_torque_not_in_tolerances`）。新 `_torque_source`（l7_mass.py:151-197，run 最前面）：`_load:rated_stall_torque` INFO PASS(measured=权威值) / 取不到 → unknown；`_load:torque_source_consistent`：废弃键仍在且 ≠ 权威 → FAIL(BLOCK, measured=差值)，相等 → PASS(INFO) 提示应删，不存在 → PASS(INFO)。反例 n_l7_torque_source（写回 0.5 → −0.1 红；真数据 / 写回 0.6 绿）。
- F-L7-6 元件认领泛化：`_CLAIM_BY` + `_mass_items` 按 `components.yaml:components[].claim` 认领（l7_mass.py:553-569 注释、704-748 `claim_stems`），**不再按 category 分支**。没有 claim → `component_body:<id>` unknown（l7_mass.py:763）；规则用不了 → `component_claim_rule` unknown（773）；`component_qty`：认领实例数 == qty → PASS(INFO)，≠ → FAIL(BLOCK, measured=实例数)（779）；qty 不是整数 → `component_qty` unknown（744）；qty=0 → 不装、不要求质量（components.yaml speaker/filament 的 gate_schema_note 要的就是这个）。宿主 body 改为"housed_by 全部实例的 body，去掉左右镜像后仍不止一个连杆 → 不猜；否则每个实体按最近 body 原点落到左/右"（l7_mass.py:663-682；以前只取第一个实例的 body，右腿件会挂到左 body）。`_layer:component_claims` INFO 簿记（831）。反例 n_l7_component_claim。
- §2.6 泛化：`mirror_body_pairs`（l7_mass.py:438-459）由 frozen left_/right_ 同名对推（left_ 数 == right_ 数 == 配到的对数，双方 body 都在 MJCF 里），criterion 写"配到的对数 == 数据声明的对数"；`_home_pose`（l7_mass.py:1279）：任一关节缺 home_rad → `_load:home_pose_declared` unknown、home=None → `static_torque_home` / `com_in_support_home` unknown（以前 `or 0.0`）；`_stability`（l7_mass.py:1533-1664）落地件只认 `parts.yaml:ground_contact: true`，没有声明 → `support_polygon` unknown；筋容差从 `tolerances.yaml:load.min_section_and_rib.rib_declared_vs_measured_tol_mm` 读（l7_mass.py:1830-1850，缺/无 src → `_section:rib_declared_tol` unknown 且每条 rib_thickness/rib_length unknown）；密度（603）/ 稳定裕度（1585）/ 筋容差（1847）src=assumed → `threshold_assumed:<键>` FAIL(WARN, measured=阈值)，src 无 → 引用它的判据 unknown；`part_mass` / `com_in_support_*` / `rib_*` / `static_torque*` 的 criterion 都带 src 标签。**数值一个都没改**（1.27 / 3.0 / 0.1 / 0.6）。反例 n_l7_generalized。
- 模块 docstring 加 2026-09-13 一节。

### 数据（只加字段；唯一删除是 yaml 自述要删的镜像键）
- `components.yaml`：顶部注释（:10-15）说明新字段 `claim`；给现有 7 条元件补 `claim`（servo_s288:37 `by: mjcf_servo_geom, body_from: mjcf_servo_host`；bearing_6704zz:134 / 6700zz:177 / 6702zz:217 / 22x16x4:254 `by: envelope_ring`；battery_3s:286 `by: envelope_box`；original_prints:774 `by: placed_prefix, pattern: orig_, body_from: mjcf_part_mesh, mass_estimate_from_geometry: true`）—— 规则内容 = 原层代码四个分支逐条搬入，没有新匹配。**字段说明**：`claim.by ∈ {mjcf_servo_geom（placed/servo__<body>_<drives>，来自 MJCF body.servos）, envelope_box（envelope_mm.v 的 a×b×c 反算体积+包围盒 ±0.05 / 1%）, envelope_ring（内径×外径×厚反算环体积+包围盒）, placed_prefix（placed 名前缀 claim.pattern）}`；`claim.body_from ∈ {housed_by（缺省）, mjcf_servo_host, mjcf_part_mesh}`；`claim.mass_estimate_from_geometry: true` = mass_g 取不出数时按体积×密度推 INFO 值（不冒充声明，仍算缺口）。其余 14 条（主板/IMU/摄像头/屏/麦/小板/降压/总线板/插头/线/开关/绑带/耗材/喇叭）**没有 claim**：placed/ 里没有它们的实体，层现在对它们发 `component_body` unknown（speaker qty=0 除外）。未删任何字段。
- `tolerances.yaml`：删 `load.servo_output_bearing.rated_stall_torque_Nm`（:1142 处留注释说明；它自己的 duplication_note 写明应删）；`load.servo_rated_torque_Nm.mirrored_into`（:1422）改为"已删除"；新增 `load.min_section_and_rib.rib_declared_vs_measured_tol_mm: {v: 0.1, src: assumed, src_note, applies_to}`（:1181-1186，从 l7_mass.py 的 ±0.1 逐字搬入）。
- `parts.yaml`：L06 加 `ground_contact: true`（:215，值来自本条 material "TPU 95A" + role "TPU 鞋底"，逐字搬入）。
- 未动：README / CHANGELOG / negatives/README / data/README.md / frozen.yaml / bench.yaml（bench.yaml:248 那句"层现在读的是它的镜像 load.servo_output_bearing.rated_stall_torque_Nm"已过时，待数据侧改）。

### 反例（tools/gate/negatives/）
- 新增 `_l7_fixture.py`（共用夹具，不是 n_*）：`complete_inventory` 把真数据补成质量清单完整的对照 —— **只删不编**：无实体无质量的 14 类元件 qty=0；6702 / 22x16x4 / original_prints qty=0 并从替身 placed/ 拿掉实体；6704 / 6700 housed_by 收成单一宿主件。真数据下 F-L7-4 让整机判据全 unknown，对照必须用它。
- 新增 n_l7_static_torque_home_and_range（17 s）、n_l7_mass_gap_unknown（17 s）、n_l7_component_claim（17 s）、n_l7_torque_source（1.7 s，快跑：parts 清单置空）、n_l7_generalized（17 s）。修前 5/5 失败、修后 5/5 过，均通过 runner.check_contract。
- `n_l7_mass_torque.py` 改用夹具（对照 / ①③ / ② 三次 run 都过 `complete_inventory` + `placed_stand_in`），仍过（24.6 s）；n_l7_inertia_tensor_all / n_l6l7_retired_not_stale / n_core_severity 复跑通过。

### 子集跑 `gate.py --layers 7 --skip-negatives`（scratchpad gate_L7，9.97 s）：L7 310 条 / 135 红（基线 build #7）→ **348 条 / 179 红**
- +14 `joint:*:static_torque` unknown、+14 `static_torque_home` unknown、+1 `com_in_support_home` unknown、+1 `head_mass_budget` unknown、`com_in_support_zero` 由 WARN-FAIL(0.843) 变 unknown(WARN)：全部因质量清单 32 项缺口（4 种轴承跨 body / 无质量、13 类电子件无实体无质量、original_prints 无标量质量且 qty 4≠3、connector_xt30 qty "2 对"）—— **预期结果，不填数**。detail 里保留下界（neck_pitch 扫描峰值 26.1%、home 4.5%；home 重心余量 12.811 mm；头部预算上界 509.2 g）。
- +12 `component_body` unknown（sbc/imu/camera/display/mic/sensor_small_boards/buck/bus_adapter/cable/switch/strap/filament：没有 claim，以前静默消失）；−2 `world_placement_for_mass`（sbc / connector_xt30 改由 component_body / component_qty 表达）；−1 `component_mass`（speaker qty=0 不再要质量）。
- +1 `original_prints:component_qty` FAIL(BLOCK, measured=3)：qty 4 但 placed 只有 3 个 orig_*（components.yaml 自己写了 jaw_soft 没有独立实体）—— 真的数据/CAD 不一致，红得对；+1 `connector_xt30:component_qty` unknown（qty "2 对" 不是整数）。
- +3 `threshold_assumed:*` WARN（density 1.27 / static_stability_margin 3.0 / rib 容差 0.1，measured=阈值）。
- 新绿：`_load:rated_stall_torque` 0.6、`_load:torque_source_consistent`（废弃键已删）、`_load:home_pose_declared` 14、`_mjcf:mirror_body_pairs` 5（== 声明 5 对）、6 条 `component_qty` PASS（servo 14 / 6704 4 / 6700 2 / 6702 2 / 22x16x4 1 / battery 1）。
- 其余类别（load_criteria 18、inertia_criteria 15、torque_criteria 14、forcerange 14、load_path_* 33、rib_* 3、density_per_material、static_torque_margin…）与基线一致。

### 数据变更汇总（第二批，全部文本级编辑、注释未丢、`yaml.safe_load` 通过）
- `assembly.yaml`：步 id/seq/subassembly_of、`subassemblies`、`tool_states.*.after_step/after_motion/side/subassembly_of`、步 8 重排（D15）。
- `keepouts.yaml`：KO01 `window_face_grid_mm/window_face_probe_offset_mm`、KO02/03/04/10/13 `checked_in_layer(+covered_by_subjects)`、KO06 `servo_frame_of_joint`、KO07 `axis_joint`、KO11 `probe_zone_z_mm`、KO12 `exempt[]`、KO15 `fov_deg/apex_world_mm/optical_axis_world`、KO06/07/09 `facet_noise_mm3`。
- `components.yaml`：`claim`（7 条，L7 agent）、`envelope_solid`（7 条）。
- `frozen.yaml`：`joint_axes[].target_range_deg`（15 条，L6 agent）、`battery_bay_26.measured_on`、`h01_sbc_posts.measured_on/post_axis_world`、`imu_pose.measured_on/mount_hole_d_mm`。
- `tolerances.yaml`：32 处 `src: assumed`（A）、`frozen_dimension_mm`、`envelope_vs_placed_mm`、`load.min_section_and_rib.rib_declared_vs_measured_tol_mm`（L7）、删 `load.servo_output_bearing.rated_stall_torque_Nm`（yaml 自述要删）。
- `parts.yaml`：L06 `ground_contact: true`。`printability.yaml`：`no_support_zones.tiers/per_feature_overrides`、`parts.<件>` 逐件切片配置、`slice_run` 重生成。`features.yaml`：L01-F13/L03-F05 `mating_shells_export_local`、H01-F06 purpose 订正。`data/README.md`：刀心不是坐面一句（A）+ 本批字段说明。

### 全量跑（第二批后，`docs/build_logs/gate_2026-09-13_第二批后.log`；归档 `tools/gate/out/archive/scorecard_2026-09-13_第二批后.json`）
- 64 个反例全过（缓存 key 变了重跑，≈217 s）；总 ≈1010 s。基线 85 PASS / 351 FAIL / 7 NOT_RUN / 1 RETIRED → **107 / 357 / 6 / 1**，仍 BLOCKED。
- 逐层（判据数/红）：L0 96/14 → 98/14；L1 184/53 → 219/58（+removable WARN 15、forbid 真红 4：H01/L01/L03/H02）；L2 558/247 → 560/248；L3 92/37 → 134/38（envelope/frozen 实测全绿、SNAP unknown、14 类未建模元件 unknown 照旧）；L4 148/54 → 362/67（present_covers_derived 141 条 + 5 WARN + KO02/03/04 汇总格）；L5 533/315 → 545/296（坐面起点改 seats 后 head_seat unknown 转有数，+31 threshold_assumed WARN）；L6 73/56 → 78/58（collision_free_range 绿、gen_range_covers_target/undeclared_zero_gap_pair WARN；BLOCK 不降因 target = 上游）；L7 310/135 → 348/179（质量缺口 → 整机判据 unknown、component_body unknown 16、original_prints qty 4≠3）。


## 2026-09-13 —— 任务 A：S288 舵机参数全部改手册值（CAD 源码 + 检查脚本；**Gate 数据一个字没动、发布目录未同步、Gate 未跑**）

依据：手册尺寸图 p2（`docs/reference/s288/S288使用手册_含尺寸图_2026-09-10.pdf`，主线程 400 dpi 读数）+ 用户 2026-09-13 拍板"我自己量的不准，所有参数按手册"；分度圆 09-13 实物定案 Ø10.5（旧 r4.75 摆臂扣法兰只对得上 1 孔）。
产物：`cad/duck_s288/*.stl` + `placed/` + 两份 audit json = **build #8**（`docs/build_logs/build_2026-09-13_manual_dims.log`：静态 0、mechanical_audit PASS、EXIT=2 = B01 未闭环预期；扫掠 268 条与 build #7 同集合，全在 MJCF 极限角、旧检查域内 0）；
三只读检查 + 自证 `docs/build_logs/checks_2026-09-13_manual_dims.log`（idlercheck 锁死 0 / asmcheck 14 颗全可抽、-y 走廊两条同 build #7 / audit_screws 问题数 6 同 build #7 / 6 孔拟合 12 组全 r=5.250、口袋 10.30/13.30、法兰让位 ≥Ø14.6）。
**冻结项没动**：电池仓 B01 常量、关节轴线（kin）、壳柱 SHELL_BOSS、香橙派立柱；测试台 `tools/cad/testbench.py` 只改常量与注释，**未重跑**（`cad/testbench_s288/` 仍是 09-13 白天的 STL）。

### `duckstructure/s288.py`（参数字典 S，全部标"手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）"，旧实测值留作历史）
| 行 | 键 | 旧 → 新 | 来源 |
|---|---|---|---|
| 16 | `T` | 19.9 → **20.0** | 手册 p2 轴心段厚 |
| 17 | `T_lo` | 22.8 → **23.0** | 手册 p2 接线端厚 |
| 18 | `step_z` | (-7.0, -12.0) → **(-7.0, -13.5)** | 手册 p2 像素读数：厚段自接线端 ≈10.8（z≈-13.7）、斜坡 ≈6.4、薄段自 z≈-7.3；取 -13.5 比 -13.7 保守 0.2 |
| 22 | `flange_d` / `flange_h` | 13.9 / 2.9 → **14.0 / 3.0** | 手册 p2 法兰 Ø14 高 3 |
| 23 | `horn_r` | 4.75 → **5.25** | 手册 Ø10.50 + 09-13 实物定案；⛔ 争议块（旧 20–26 行）删除，换成 3 行定案说明（24–26 行）指向 `docs/reports/分度圆Ø9.5还是Ø10.5_2026-09-12.md` |
| 28 | `rear_boss_d` / `rear_boss_h` | 13.8 / 2.9 → **14.0 / 3.0** | 手册 p2 副轴 Ø14 高 3；端面 -13.0 与厚段背面 -13.0 齐平（20+3=23） |
| 2–6 | 模块 docstring | "标 [量] 的值来自手册图纸像素反推或纯假设" → 本体/法兰/副轴/分度圆全按手册，[量] 只剩插座/插头 | |
| 149 | 新增 `x_env_boss_face()` | = `x_idler_face()−1.0` = -14.0（servo_envelope 副轴让位外端面；c 在公式里抵消） | 供 lib 推 X_ENV_BOSS |
| 143–150 | 站位注释 | 12.85/-9.95/-12.85/-14.95 → 13.0/-10.0/-13.0/-15.0，标旧值 | |
| 102–109 | `servo_envelope` docstring | 说明包络仍按保守台阶 (2,-1)，不因手册有了 step_z 放松 | |
不动：W/L/top/horn_hole_d/horn_center_d/idler_*/mnt_*/horn_depth/mnt_depth/conn_*/plug_*/clr/wall/plate_t/stub_*/brg_*/mass_g（19.5 g 官方参数表）。

### 由此推出的站位变化（全部由 S 推导，不再写死）
法兰面 12.85→**13.0**；薄段背 -9.95→**-10.0**；厚段背/惰轮端面 -12.85→**-13.0**；载体背板内面 XR -13.15→**-13.3**；让位体副轴面 -13.85→**-14.0**；等厚背板外表面（薄/厚）-13.25/-16.15→**-13.3/-16.3**；躯干顶板 BZR0/BZR1 144.65/147.65→**144.8/147.8**；甲板 DECK_TOP/BOT 121.25/119.15→**121.2/119.0**（手册尺寸下前板底正好 = 原版甲板底 119.0，不再削 0.15）；N03 A 盘坐面 YRM_A2 11.2→**11.35**；N02 顶盘底 224.2646→**224.115**。

### `duckstructure/lib.py`
- L16 `XR` 注释 -13.15→-13.3；**L17 新增 `X_ENV_BOSS`**（-14.0）；**L18–20 新增 `x_bshell_out(t, thick)`**（等厚背板外表面 -13.3 / -16.3，沉孔坐面从它推）。
- L33–37 `ANK_X0/ANK_X1b/ANK_X1`：写死 -17.4/-21.4/-21.6 → 从 X_ENV_BOSS 推（-14.0−3.0−0.4 = **-17.4，手册尺寸下与旧值相同**）+ 断言 ==-17.4/-21.4（`tools/cad/ankle_split.py` 分体站位按这两个数写死，ANK_X0 再变要一起改）。
- L150–151 `BZR0/BZR1`：写死 144.65/147.65 → `pt(sfw("trunk_base",0), XR)[2]` = **144.8** / +PLT = **147.8**（定义移到 `pt()` 之后）。电池仓 BZ1=146.9 不动。
- L234 `CARD_T`：3.1 → `(-T/2 − XR) − 0.1` = **3.2**（垫片卡缝 3.3；目前无载体用 card 模式）。
- L346 `bearing_driven_clearance`：环中心写死 14.875 → `xf + 2.025`（x 12.8..17.25）。
- **L475–484 `back_shell`：新增 `BSHELL_EPS=1e-3`**，板内腔按 `servo_envelope(grow−1e-3)` 做、调用方再减 servo_env(grow) 定最终内腔面。原因：手册尺寸下让位体副轴面落在 -14.0 这种站位后 L04（原版板 + 背板 + 让位刀共面/共柱面）manifold 输出经 float32 焊合仍剩 1 条非流形边 + 1 片 -0 体积碎片，`checks.export` 的 L04 严格断言不过（首次 build 失败，见日志开头被覆盖前的记录：本节"过程"）；退 0.001 后 L04 退化 0 / 非流形 0。外表面只薄 0.001。
- L430 `flange_relief` 默认 c：0.2 → **CLR=0.3**（Ø14.4→**Ø14.6**，题面"法兰让位孔 ≥ 14+2·clr"；影响 L01/L02/L04/L05/N02 的法兰让位圈）。
- L566 `mnt_cut2` 新增断言 `cb_x0 is None or cb_d > 0`（本任务第 3 次 build 时三处 cb_d 相减写反成负值、沉孔被静默跳过，audit_screws 才暴露 L04 下排叠厚 4.70；已改正并加断言防复发）。
- 注释更新（旧数→新数或标"旧值"）：SLIDE_CLR、ANK 注释、P.brg_yaw 注释、carrier docstring/站位注释、mount_cut_local、原版重雕段注释、driven_patch、idler_hub/idler_bore/idler_keep、flange_slot、seat_ring_clearance/zone。

### `duckstructure/neck.py`
- L16 `NK_LIP_Y`：写死 (10.25, 12.35) → `(T/2+CLR, x_flange_face()−0.5)` = **(10.3, 12.5)**。
- **L20 `NK_IDLER_BORE` 15.0 → 16.0**（报告 §定案后 b）；L21–30 注释按 r 8.0 重算：垫柱余量 0.466（报告 0.47）、Ø4.4 沉窝层韧带 0.766（报告 0.77）、Ø2.4 孔层 1.766 —— **建完 STL 射线实测 0.767 / 通孔 r 7.990..8.000**（checks 日志 [自证补充]）。
- **L105 `NP_IDL_HUB_D` 14.0 → 15.0**（报告 §定案后 a；`lib.idler_hub` 断言：孔外缘 6.55 → 毂边 0.95 ≥ 0.8；Ø14 时 0.45 会响，已验证会响）。
- L147–150：`YRM_A2` 写死 11.2 → `法兰面世界 x(8.1) + YRM_A_STACK(3.25)` = **11.35**；`YRM_BORE` 起点跟随；`YRM_A0` 6.4 不动。F17 叠厚保持 3.25（audit_screws jaw_soft 法兰孔 3.25 ✓）。
- L191 ringA 让位孔：`flange_d+0.3`（Ø14.3）→ `flange_d+2·CLR`（**Ø14.6**）。
- L224 N03 上排沉孔：`cb_d=6.75`（沉到 -13.25）→ `x_bshell_out(PLT) − (−20)` = **6.7**（沉到背板外表面 -13.3，叠厚仍 3.3）。
- 注释：NP_CB_D（5.25+Ø4.4 → Ø15.0）、Ø15.05 通道对螺丝头 0.275、N02 叠厚 3.40（旧 3.55；M2×6 咬 2.6）、起子通道 14.7、A 端法兰面 8.1 = 原版 XL330 接口面 8.10、N04 注释 227.6、xr0 -13.3。

### `duckstructure/trunk.py`
- L60–63 `DECK_TOP/DECK_BOT`：写死 121.25/119.15 → 从髋偏航舵机帧推 **121.2 / 119.0**（前板底 = 法兰面 118.5 + 0.5；L01 毂顶 118.5 → 间隙 0.50 不变）。`CLR` 加进 import。
- **L164 `SHR_IDL` 毂径 14.0 → 15.0**（报告 §定案后 a；皮层填孔盘 sd 14.6 保留，理由写在注释）；mechanical_audit T03 惰轮毂 306.8/307.3 mm³（旧 243.3/243.7）。
- 注释：法兰 Ø14/118.5/121.5、座环 z 114.5..119.0、背板 144.8..147.8、y0..y1 13.0..16.0、SLIDE_CLR=0.4（13.4；旧注释错写 0.2/13.05）、壳内壁 -17.1、惰轮螺丝 5.40/咬 2.6、N01 Ø16。

### `duckstructure/legs.py`
- L9–10 新增 `L01_SEAT=19.2`（原版杯壁外表面 = 横滚舵机顶端 112.0+CLR，不随 T 变）/ `L01_STACK=5.35`；L28 沉窝深 `1.0` → `L01_SEAT − (xf+5.35)` = **0.85**（叠厚保持 5.35，M2×8 咬 2.65；mechanical_audit 6×16 全 5.35）。
- L30 L01 上排沉孔 `cb_d=1.25` → `x_bshell_out(3.0) − (−14.5)` = **1.2**（沉到 -13.3，叠厚 3.3）。
- L41 挡肩薄片补切 wbox x1 写死 18.8 → `pt(Rs, xf−0.1)[0]+0.05` = **18.95**（座孔起点随 xf 外移 0.15，薄片 0.15→0.3，不补切会剩 0.1）。
- L91–92 / L103 垫管座：`x_in=-13.0, x_out=-14.95` / Ø5.5 让位 -14.95 → `x_in = x_bshell_out(3.0)+0.3`（-13.0）、`TUBE_OUT = -T/2−5.0`（**-15.0**）。叠厚 5.5 不变。
- **L125 新增填柱**：原版 L04 踝轴心的 XL330 阶梯定位孔（Ø5.5 台阶面 x=-14.0 / Ø2.7）用 Ø6 填掉（它整个落在背板副轴盘 + Ø9.95 轴颈里；台阶面与 X_ENV_BOSS=-14.0 严格共面是 L04 退化的诱因之一）。
- L130 L04 下排沉孔 `cb_d=1.55` → `x_bshell_out(3.0,thick=True) − (−17.7)` = **1.4**（沉到 -16.3，叠厚 3.3；audit_screws ankle 背角孔 4 孔 3.30 ✓）。
- L134–135 踝轴颈起点写死 -13.85 → `X_ENV_BOSS−1.0` = **-15.0**（退进副轴盘 1.0，真·体积重叠；起点 = -14.0 时端面与盘面差 2e-15 留退化）。
- L162 L05 沉窝起点写死 19.55 → `xe`（= 法兰面+6.7 = **19.7**；不改会留 0.14 封闭空腔）；叠厚 5.5 不变。
- L23 `flange_relief` 调用不变（默认 c 变 0.3）。docstring 数字更新。

### `duckstructure/head.py`
- **L27–31 新增 `H01_FOOT_RIM_R = horn_r+2.1+0.05 = 7.4`；L97–99 右前脚 (49.1, 9.35) 承压圈内侧削到离横滚轴 r≥7.4**（报告 §定案后 c：A 端 6 颗 Ø4.2×45 起子刀路 0° 那条擦到承压圈 1.19 mm³）。刀沿横滚轴长 8（首版写 2.1 只盖住脚盘中间一段，mechanical_audit N03 tool corridors 仍 FAIL 0.19 mm³，第 2 次 build 才过）。实测：脚盘剩 42.7 mm³、圈内缘 r 7.391、孔边到削面 1.15；6 条刀路对 H01 全 0。
- L112–113 新增 `H02_POST_L/H02_PLATE_T/H02_CB = 3.2/3.0/0.6`、`H02_ARM_X=(-18.15,-12.0)`；L121–130 H02 压板/垫柱/头窝全部从舵机薄段背面世界 x（-14.9）往 −x 推：压板 -18.05..-21.05 → **-18.1..-21.1**，垫柱 3.25 → 3.2+0.05 压进板，头窝 -20.76 中心 → 板外面沉 0.6。叠厚 5.60 / 臂 6.10 不变（mechanical_audit 2×16 全 5.60 / 6.10）。
- 注释：法兰面 8.1 / 薄段背 -14.9 / 厚段背 -17.9；前立板 Ø22.78 孔说明 r 7.4 / Ø>14.8。

### `tools/cad/*.py`（检查脚本）
- `assembly_audit.py:45–53`：轴承 -x 面 12.85 ×6 → `XF = D.S['T']/2 + D.S['flange_h']`（13.0）；踝 6700 -21.4 ×2 → `D.ANK_X1b`。
- `mechanical_audit.py`：L01 叠厚 5.35 → `D.L01_STACK`；N02 期望 3.55 → `NP_CHK_OUT − NP_CB − xf` = 3.40；H02 期望 5.60 → `H02_POST_L+H02_PLATE_T−H02_CB`；H02 臂 6.1 → 由 `H02_ARM_X` 推；N03 起子刀起点 `xf+3.25` → `YRM_A2 − 舵机心 x`；T03 毂探针中心 -14.5 → `x_idler_face()−1.5`；horn_r 处已读 `S['horn_r']`（确认，只改注释"09-13 定案"）。
- `idlercheck.py:56–66`：外圈取样半径 `rear_boss_d/2−0.8`（5.25 时 6.2 落进孔带 3.95..6.55）→ `(rear_boss_d/2 + horn_r + idler_hole_d/2)/2` = **6.775** + 断言在孔带外 ≥0.1、盘内 ≥0.1；docstring "争议" → "定案"。
- `testbench.py:136–138`：`HORN_R_ARM = 5.25` 写死 → `S["horn_r"]` + 断言 ==5.25；文件头/注释站位数更新（12.35→12.4、-12.85→-13.0、-15.95→-16.1、14.85→15.0、17.35→17.5、副轴 Ø14×3）；**未重跑，STL 未变**（文件头已写明重跑会变什么）。
- `asmcheck.py:49`、`coupons.py:9/72–73`：注释（Ø15.8→Ø16、r4.75→S、-12.85/-9.95→-13.0/-10.0）。
- `V2_S288版发布产物/06_CAD源码/audit_screws.py`：未改（全部读 S）。

### 过程（5 次 build，每次 ≈8 min）
1. L04 export 断言"退化面/非流形共边"失败 → 定位到 -14.0 站位共面（见 back_shell / L04 填柱 / 轴颈起点三条）。
2. mechanical_audit FAIL `N03 tool corridors` 0.19 mm³（H01 承压圈刀长写短）。
3. 通过，但 audit_screws 暴露 L04 下排叠厚 4.70（三处 cb_d 符号写反、沉孔被静默跳过）。
4. 通过；自证发现法兰让位 Ø14.4/Ø14.3 < 14+2·clr → 放到 Ø14.6。
5. **最终**：静态 0、PASS、三检查 + 自证全过。日志文件是第 5 次的。

### 螺丝叠厚变化（audit_screws，对 build #7；其余各组不变）
F12b 颈俯仰惰轮 5.55→**5.40**（M2×8 咬 2.6）；F11 N01 上排 5.20→**5.30**（M2×8 咬 2.7）；F14/F14b N02 两侧 3.55→**3.40**（M2×6 咬 2.6）；壳柱"背角孔 shell_L/R"伪配合行 3.47/3.70→3.42/3.65（同 build #7 的 audit 噪声）。F02 5.35 / F03 3.3 / F09 3.3 / F10 5.5 / F15 3.3 / F17 3.25 / F18 5.6 / F04/F06/F08 7.5 / F16 3.5 不变。

### Gate 数据后续要跟（本任务未做）
`features.yaml` 8 条 `horn_hole.pitch_r_mm 4.75`→5.25（`tools/gate/retarget_horn_r.py --old 4.75 --new 5.25 --apply`）、所有 12.85/-12.85/-9.95/-13.15/144.65/121.25/119.15 站位、N01-F?? 通孔 Ø15→16、T03/N02 毂 Ø14→15、F12b/F11/F14/F14b 叠厚、`frozen.yaml` 若记了顶板 z、`fasteners.yaml:tool_access` 坐面 y/z（法兰面外移 0.15）。发布目录 `V2_S288版发布产物/` 仍是 build #6。

## 2026-09-14 —— 复审修复 r1（对手册尺寸 build #8 的 2 BLOCKER + 6 MAJOR；MINOR/NOTE 未动）
build：`docs/build_logs/build_2026-09-13_manual_dims_r1.log`（第 3 次 r1 build = 最终，EXIT=2 = B01 预期）；三只读检查 + 8 条复量：`docs/build_logs/checks_2026-09-13_manual_dims_r1.log`。
复量脚本 `measure_r1.py` 正文附在 checks 日志末尾（量法 = 复审同款：manifold 布尔 / min_gap / 射线）。

### 逐条：修前 → 修后（复量数字）
| # | 发现 | 改动（file:line，旧 → 新） | 修前 | 修后 |
|---|---|---|---|---|
| B1 | L01 F02 12 颗头压进髋横滚舵机顶端 | `duckstructure/legs.py:14` `L01_STACK` 5.35 → **4.0**；`:15` 新增 `L01_SCREW_L=6.0`；`:33` 沉窝深 = 19.2−(13.0+4.0) = 2.2（旧 0.85）。F02 M2×8 → **M2×6**（咬 2.0；8 > 4.0+3.0−0.3 会顶底） | 坐面 18.35，头 z 111.55..113.15 vs 舵机顶 112.0：∩ 5.646 mm³/颗 ×12，min_gap 0 | 坐面 17.0（16 向 ×6 孔 ×2 侧全 4.00），头 z 112.9..114.5：∩ 0 ×12，min_gap **0.9**；头∩L01 0；头↔6702 内圈 0.25（同一转动体、内圈端面 17.0 与坐面共面，无相对运动，记录不判） |
| B2 | T02 左壳压住 T01 轭 F12 6 颗头 + 孔内"销" | `duckstructure/trunk.py:209–215` 左壳新增 6×Ø(4.0+2·CLR)=Ø4.6 沿 +y 从轭盘外表面 16.0 下 0.5 打穿（头 + PH0 通道，同 T03 F12b 做法）+ 中心 Ø5 只切到轭外 SHELL_CLR（不在壳外开洞）；`:66` 新增 `YOKE_Z0=144.0`（`:114` 原刀里的写死 144.0 改用它） | 头∩shell_L [10.66,16.01,16.01,10.66,16.01,16.01] mm³；Ø2 螺杆∩ 0.645..0.835/孔；孔 1/2/4/5 孔心射线壳料 y 15.6..17.9；起子 Ø4.2 ∩ 3.0..4.1；+y 退 0..40 峰值 16.007 | 全部 **0**（头/螺杆/起子/滑入）；孔心射线 6 孔全空；Ø26 足印 y 16.0..16.5 内壳料 27.35 → 6.45 mm³ **全部在 z<144.0**（x 19..27，轭带下方躯干本来无料的壳壁，不是销；z ≥ 144.05 内 = 0） |
| M3 | H01 右前脚承压圈孔壁到削面 1.02 | `duckstructure/head.py:34` `H01_FOOT_RIM_R`（整圈 r≥7.4）删 → `H01_FOOT_TOOL_D = 4.2+2·KO_OVER = 4.3`；`:103–106` 脚盘只减 6 条起子刀路本身（沿 x，位置从 Rr/horn_r 推，与 mechanical_audit 同源；只有 60° 那条碰脚） | −y 壁厚 z 231.55/231.9/232.36/232.8/233.1/233.17 = 1.815/1.606/1.360/1.154/1.047/1.022；脚盘剩 42.70 mm³ | 1.849/1.602/1.397/1.312/1.308/1.311，72 方位最薄 **1.308**（mechanical_audit 6 高度 ×72 方位 1.310@z232.84,270°）；6 条 Ø4.2×45 刀路 ∩ H01 全 0，脚盘到刀路 0.0499（=KO_OVER）；脚盘剩 43.24 mm³ |
| M4/M8 | H02 压板内面离头横滚惰轮端面 0.2 | `duckstructure/head.py:121` `H02_POST_L` 3.2 → **3.4**（叠厚 5.6 → 5.8，F18 M2×8 咬 2.2）；`H02_ARM_X` 不动（臂坐面铣到 −18.1，臂叠厚 6.1 不变） | min_gap(H02, 惰轮 Ø14×3) 0.200；探针 +0.3 ∩ 2.836 mm³；端面 188 采样点沿 −x 命中 0.200 | **0.400** / 0 / 188 点全 0.400；mechanical_audit H02_stack 2×16 全 5.80、臂 6.10 |
| M5/M6 | T02 左壳与左髋偏航惰轮盘实体相交 0.019 mm³（<0.05 阈值盲区） | `duckstructure/trunk.py:200–202` 两侧壳各减同侧髋偏航舵机 Ø(14+2·CLR+0.2)=Ø14.8 × (薄段背 −10.0 → 惰轮面 −13.0−CLR) 让位柱；`:170` `HIP_IDLER_RELIEF_EXTRA=0.2`（64 边内接刀 Ø14.6 平边 7.291 实测 0.291 <0.3，第 1 次 r1 build 发现） | shell_L∩舵机 0.01859 mm³（全在副轴），Ø14.2×3.1 探针 ∩ 0.2197，min_gap 0.0，柱面 21600 点 203 个在壳内；shell_R 0.3385 | shell_L ∩ 0 / 探针 0 / min_gap **0.300**（轴向 = CLR）/ 0 点；shell_R **0.391** |
| M7 | 头横滚舵机厚段底角离冻结上头壳 0.199 | `duckstructure/lib.py:137` `SERVO_SHIFT[("jaw_soft",1)]` −1.5 → **−1.7**（沿自身轴 +0.2，世界 x 心 −4.9 → −4.7、法兰面 8.1 → 8.3；复审 fix_hint 写的 −1.4 方向反了，且 +0.1 只到 0.265——最近点斜向，每 0.1 涨 0.066）。跟着帧走：`neck.py:149` YRM_A2 11.35 → 11.55、H02 站位（压板仍 −18.1..−21.1）、H01/H03 舵机让位刀、KO01 CF4 窗。**连带**：`neck.py:221` N03 加 `flange_relief(Rr)`（原版 A 端腹板从 8.10 起，法兰面 8.3 后有 0.2 层压进 Ø14：零位 17.67 mm³、head_roll ±25° 20 姿态，第 1 次 r1 build 发现）；`head.py:150–153/167` H03 加 `minkowski_box(H02, ±CLR)` 让位刀（舵机 +0.2 后 H02 下垫柱底落到原版地板垫台立壁旁，min_gap(H02,H03) 0.244 → 0.069，第 2 次 r1 build 发现；修后 0.300） | min_gap(舵机, 上头壳) 0.1993；body_prism(c=0.2/0.25/0.3) ∩ 0.0027/0.0113/0.0294 mm³ | **0.3313**；c=0.2/0.25/0.3 ∩ 0.0000/0.0008/0.0056（0.3 下 0.0056 mm³ 是斜向底角，min_gap 已 ≥0.3）；× H03 0.300；F17 叠厚 3.25 不变；head_roll_B 轴向隙 bearing_to_N03 0.12 / N04_to_N02 0.60 不变 |

### 检查脚本
- `tools/cad/mechanical_audit.py`：`:55` L01 咬入 `8−v` → `D.L01_SCREW_L−v`；`:58–66` 新增头 vs 髋横滚舵机（∩≤0.05 且 min_gap ≥0.3）；`:76` 新增 `_review_r1()`（H02↔惰轮 / 两壳↔髋偏航惰轮 / 舵机↔上头壳 / F12 头×T02 静态+滑入+起子+销 / H01 脚 4 最薄壁 ≥1.2）挂在 `out['review_r1']`；H03 拆卸路径 hosts 加 `head_clamp`；新增 `H03 vs H02 nominal clearance ≥0.3`；汇总行 `[09-14 复审不变量]`。
- `tools/cad/idlercheck.py:68` 取样圈加盘边 r=rear_boss_d/2=7.0（复审建议）。副作用：踝的 L07 后臂在 r 6.775/7.0 两圈上各 12 点命中 → 现在显示为"从动件 ankle_rear_arm 差+8.40"（6700 轴承在中间，结论不变 ○）。
- `tools/cad/assembly_audit.py:57` 注释 −4.9/−1.5 → −4.7/−1.7（代码本来按帧推）。

### Gate 数据（本次只改了三处值，其余列为待跟）
- `fasteners.yaml` F02：`spec` M2×8 → **M2×6**（保留 `spec_was_until_2026-09-14`），`stack_mm.v` 5.35 → 4.0，`engagement_mm.v` 2.65 → 2.0；F18：`stack_mm.v` 5.6 → 5.8，`engagement_mm.v` 2.4 → 2.2（src_note 各带日期）。`counts.by_spec` 直方图（采购清单原文）未动。
- `frozen.yaml:619` SERVO_SHIFT `jaw_soft_1` −1.5 → −1.7（src_note 写明理由；`joint_axes_rule` 允许沿轴平移）。
- 待跟（未做）：`features.yaml` T02 新增 6×Ø4.6 通孔特征 + 两壳惰轮让位、L01 沉窝 0.85→2.2、H02 垫柱 3.4、H03 新增 H02 让位；`features.yaml:995`/`keepouts.yaml:KO01 CF4`（头横滚舵机 +0.2）；`fasteners.yaml` F02/F17/F18 `tool_access` 坐面坐标（F02 坐面 18.35→17.0、F17 坐面 11.35→11.55）；`assembly.yaml` T02 装配序备注（F12 头露在轭外、壳上 6 孔对头）。

### 未做 / 观察
- 复审 MINOR/NOTE 未动。
- L01 导出 STL 仍 退化面 34 / 非流形边 30（洞边 0，与 build #8 相同，不是本次引入；只有 L04 有严格断言）。
- 扫掠 268 个相交对/姿态、旧检查域内 0，与 build #8 同集合（全在 MJCF 极限角）。

## 2026-09-14 —— 复审修复 r2（对 r1 build 的 7 条 MAJOR；MINOR/NOTE 未动）
两段完成：**上一 agent**（被判停摆前）已把 7 条全部写进源码并起过一次通过的 build（日志已改名保留为 `docs/build_logs/build_2026-09-13_manual_dims_r2a.log`，mechanical_audit PASS，EXIT=2 = B01 预期）；**本 agent** 核对该 build 早于最后一次源码改动（源码 04:59:36 < 起建 ~05:05）后没有重建，只跑三只读检查 + 复审同款复量，复量发现 M6 仍有一处角点接触（下），补一刀后重建 = `docs/build_logs/build_2026-09-13_manual_dims_r2.log`（最终，07:29→07:35；18 件洞边 0 实体 1、静态 0、扫掠 264 个相交对/姿态、旧检查域 0、mechanical_audit PASS、EXIT=2 = B01 预期）；三只读检查 + 7 条复量：`docs/build_logs/checks_2026-09-13_manual_dims_r2.log`（复量脚本 `measure_r2.py` 正文附在末尾）。

### 逐条：改动（file:line，旧 → 新）与 修前 → 修后（复量数字，量法同复审）
| # | 发现 | 改动 | 修前（复审） | 修后（复量，最终 build） | 谁做 |
|---|---|---|---|---|---|
| M1 | T02 左壳 F12 6×Ø4.6 头孔在 1.4 壁上留 0.655 韧带 + 悬挂中心岛，孔到脖子开口边 0.86..0.90 | `duckstructure/trunk.py:171–185` 新增 `SHL_YOKE_LOBE_R=8.42 / SHL_YOKE_OPEN_R=8.82 / SHL_YOKE_XLH=(6.0,2.2) / SHL_YOKE_OPEN_X0=21.0`（从颈俯仰帧 + 原版 XL330 孔推）；`:224–236` 把原版当 XL330 舵盘用的圆 lobe **整块开掉**（[圆 Ø17.64 ∩ x≥21.0] ∪ 方块 z≥轴心，沿 +y 从轭盘外 15.5 打穿到 28），保留 6 颗头的 Ø4.6 通道刀（180° 那颗跨左界），中心 Ø5 销刀删除。复审 hint 的 Ø15.1 通口 + hull 预检仍剩 0.4..0.9 弧带，所以整块开 | y 16.55..17.85 截面：孔间 web 0.655、孔边到开口边 0.84..0.90、中心岛有料 1.00 | 6 个截面：开口核心有料 0.000 mm²、外圈带 r 8.82..10.02 有料 = 壳体本体（θ −80..115° 带内 0 = audit band_fill 0.000）、左缘最窄 1.98..2.04；轭区 ±3 内 <1.2 宽的只有开口圆与壳壁斜边相交处的**尖角尖端** 0.34..0.42 mm²（(24.3,143.6)，该壁 z 143.5 处宽 5.1，非 web/弧带）；F12 头×T02 静态 0 / 起子 0 / +y 滑入 0..40 峰 0 / min_gap(头,T02) 0.300 | 上一 agent |
| M2 | L03 膝环座端面外缘 ↔ L04 内角 0.014 全程 | `duckstructure/lib.py:559–571` `sweep_of` 新增 `mink=True`（先 `minkowski_box(±grow)` 连续膨胀再转，各向 ≥grow；DILATE6 在凸角只有 grow/√2、角点 0）；`duckstructure/legs.py:147` 膝反向扫掠 `grow=0.4` → `grow=0.4, mink=True` | 零位 0.0293；−90..60 每 1° 0.0136..0.0141；L04 在 x 17.0..17.45 × r 15.45..16.0 环内有料 1.887 mm³ | 零位 **0.400**；每 1° 全程 0.400（min=max）；环内有料 0.351 mm³（剩的在 r≥15.9 的 0.4 带外）；∩ 0 | 上一 agent |
| M3 | L07 内孔 r 8.0 段与 L04 副轴凸台盘同径对接，零位 0.0 | `duckstructure/legs.py:172` 踝反向扫掠 `clr = sweep_of(ll, ..., grow=0.4)` → `mink=True` | 零位 0.0000、∩ 0；±35° 每 2.5° 恒 0；L07 +0.05 ∩ L04 0.1449 mm³；L07 在 x −17.45..−17.0 × r 7.9..8.6 有料 14.3/16.3 mm³ | 零位 **0.400**；每 2.5° min 0.3553@+2.5°（audit 0.353@8.75°）；+0.05 ∩ 0；环内有料 4.08 mm³（全在 r≥8.4） | 上一 agent |
| M4 | L04 踝背板顶边 ↔ L05 0.2 厚唇同角对接，零位 0.0013、多角 0 | 同上一刀（L05 与 L07 同一 `clr`） | 零位 0.0013；±35° 每 1° −34..−20 多角 0；+0.05 ∩ 0.0015 mm³ | 零位 **0.5642**；每 1° min 0.5616@−20°，为 0 的姿态 0/71；+0.05 ∩ 0 | 上一 agent |
| M5 | H03 A 端下半座仍原版 r 10.892，N03 A 盘径向 0.092 | `duckstructure/head.py:24–29` 新增 `H03_SEAT_A_X0=6.0`（原版 A 半座槽起点 = r 9.3 唇的 +x 面，顶点实测）+ 断言 `YRM_A0 ≥ +0.3`；`:166–167,176` `seatA = cyl(P['seat_d'], x 5.98..YRM_A2+0.1)` 与 seatB 一起 diff（不动 x 5.4..6.0 的唇） | A 座 x 6.5..9.8 下半圈 r 10.892..11.048；N03×H03 零位 0.0922、±25° 最小 0.0851；yrm +0.1 ∩ 0.74、+0.2 ∩ 12.4 mm³ | x 6.5/8.0/9.8 下半圈 r 11.085..11.088 = **Ø22.170..22.176**（B 座同值）；零位 **0.2847**、±25° 每 1° 最小 0.2717@+14°；+0.1 ∩ 0、+0.2 ∩ 0；N03 A 盘 r 10.787..10.800 → 径向 0.285 | 上一 agent |
| M6 | L04 ↔ 电池髋偏航 ±25° 名义 0.032；电池仓内挪 0.6 即相交 3.77 mm³ | ① `duckstructure/legs.py:148–153` 扫掠对象从电池名义实体改成**仓腔** `wbox(BX0..BX1, ±BAT_Y, BZ0..BZ1)`（= 电池所有可达位置，全 lib 冻结常量）+ grow 0.4（上一 agent）；② `:153` 再加 `mink=True`（本 agent：复量发现仓腔 + DILATE6 在『后口×外壁×仓底』角 (BX0,+BAT_Y,BZ0) 斜向覆盖 0，电池推到该角（坐仓底/靠外壁/顶后门，物理可达）在 −25° 与 L04 点接触 min_gap 0.000、∩ 0；L04∩(仓腔+0.4 方盒) 3.73 mm³） | 名义 0.0316@−25°；(0,+0.6,−0.6)/(0,+0.6,0) ∩ 3.77/3.67 mm³；右腿 @+25° 同值 | ①后（r2a build）：名义 0.894@−25°、11 种挪位 ∩ 0，但角位 min_gap **0.000**@−25°(−0.6,+0.6,−0.6)；②后（最终 build）：名义 **1.000**@−25°（R 1.000@+25°）；11 种挪位 × 45 姿态 ∩ 0；挪位最小 min_gap **0.400**@−25°(−0.6,+0.6,−0.6)（R 0.400@+25°）；audit 半格网 名义 1.379 / 极限位 0.537@−24.375°；L04 5497.5 → 5493.8 mm³（−3.73） | 上一 agent + 本 agent |
| M7 | L01 ring_boss 挡肩离 L02 毂端面 / 6704 内圈 0.095 | `duckstructure/lib.py:550–556` `ring_boss` 的 cut 再并一刀 `cyl(P['brg']['lip_d']=24, x xf−1.0..xf+0.05)`（r<12 让到 12.0，只留 r 12..13.575 压外圈端面，同 carrier(ring=True) 挡肩）；连带 `legs.py:39–41` 髋横滚 L02 扫掠改 `mink=True`（挡肩修后露出 DILATE6 在环座缺口 0.20/±22° 0.107） | 零位 0.0950、±22° 恒 0.095；hip +0.2 ∩ 0.11 mm³；x 12.5 r 9.5/11 有料 9/7 /72、x 12.85 r 11 3/72 | 零位 **0.5298**、±22° 每 1° 最小 0.3878@+22°；+0.2 ∩ 0；x 12.5/12.85/12.95 × r 9.5/11/12.5 全 0/72；挡肩带 (x 11..13, r≤12.5) ↔ 内圈探针 0.532、↔ L02 毂 0.995、↔ 6704 实体 0.470；挡肩仍在（x 12.85 r 13.3 50/72，audit 0.69） | 上一 agent |

### 检查脚本
- `tools/cad/mechanical_audit.py:130–` `_review_r2()`（上一 agent）：M1 6 截面开口核心/外圈带无料 + 左缘 ≥1.2；M2 膝 ≥0.3 零位 + −88.75..60 每 2.5°；M3/M4 踝 ≥0.3；M5 H03 A 座直径 = seat_d ±0.02（16 向）+ N03↔H03 ≥0.25 零位 + ±25°；M6 8 极限位 ∩ ≤0.05 + 名义 ≥0.3；M7 挡肩带 ↔ 内圈探针/L02 ≥0.5、L01↔L02 ≥0.3、挡肩存在 ≥50%。汇总行 `[09-14 复审 r2 不变量]`。
- 本 agent 补：`mechanical_audit.py:190–204` M6 再要求 8 个仓内极限位置的 min_gap ≥0.3（`shifted_min_gap_mm/_at_deg/_shift` 进 json，汇总行加 `极限位 gap`）——r2a 版只查 ∩，角点接触 0.000 漏过。

### Gate 数据
- `tolerances.yaml:conflicts.seat_d`：resolution 未解决 → **已解决（改件不改数）**，加 `resolved_on/resolved_by`，`gate_must_use.discA_mm` 0.09 → 0.285（导出件射线实测）；`fits.pla.plain_slide_head_roll`：`nominal_d_mm.src` assumed → measured（src_note 写 seatA 刀 + 实测 Ø22.17）、`radial_clearance_mm.by_journal.discA.measured_from_shell_seat_mm` 0.10 → 0.285、`gate_must_use` why/discA/verdict 改为通过、`status` below_target → in_target（+ `status_history`）。Gate 层不直接读 gate_must_use（l5_function 只读 nominal/target，该桶仍走"量纲不同 → unknown"），实测放行仍由 relations R11 的 relation_contact 量 STL。
- 待跟（未做）：`features.yaml` T02 轭区开口特征（替换 r1 的 6×Ø4.6 通孔）、L04 电池让位改仓腔扫掠、H03-F01 A 座 Ø22.18、L01 挡肩 Ø24；`relations.yaml:R11` 的实测值刷新；`frozen.yaml` 无需改（seat_d 不在冻结表）。

### 未做 / 观察
- 复审 MINOR/NOTE 未动；冻结件未动；手册值未动。
- M3 踝 L07↔L04 全程最小 0.355@+2.5°（audit 半格网 0.353@8.75°；≥0.3 判据，<0.4 名义；最近点位置未查），记录不改。
- M7 挡肩带 ↔ 6704 实体 0.470（<0.5）：那是对轴承整体（含外圈内缘倒角）量的，复审判据是对 L02 毂/内圈（0.995/0.532）；挡肩压外圈端面本来就该贴近。
- 三只读检查与 r1 完全一致（diff 只有 trimesh RuntimeWarning）：idlercheck 锁死 0；asmcheck 14 颗全可抽出、−y 走廊两条 ⛔ 同前（总线不用侧）；audit_screws 问题数 6 同前。

## 2026-09-14 复审修复 r3（主线程）—— 对 r2 build 的 1 BLOCKER + 4 MAJOR；MINOR/NOTE 未动
第三轮"几何+装配"对抗复审（R3g 几何视角 / R3a 装配视角）。build：`docs/build_logs/build_2026-09-14_manual_dims_r3.log`（静态 0 条；mechanical_audit PASS 含 `[09-14 复审不变量]`/`[09-14 复审 r2 不变量]`/新加 `[09-14 复审 r3 不变量]`；EXIT=2 仅因 B01 未闭环接口）。三只读检查 + 5 条修前→修后复量：`docs/build_logs/checks_2026-09-14_manual_dims_r3.log`。第 1 次 build 只有一条 FAIL（新判据 N03 起子刀 min_gap ≥0.15 实测 0.149817 = 0.15·cos(π/64)，64 边内接同相位）→ 阈值改 0.15·cos(π/64)−1e-4 重建一次，几何未变（第 1 次日志留在 scratchpad/build_r3_try1.log，会清）。

### 逐条：改动（file:line，旧 → 新）与 修前 → 修后（复量数字，量法同复审 evidence）
| # | 发现 | 修/豁免 | 改动 | 修前 | 修后 |
|---|---|---|---|---|---|
| R3a BLOCKER | H02 连接臂两颗 M2×10 过孔不贯通（刀只到 −12.5，臂端 −12.0，留 0.5 膜） | **修** | `duckstructure/head.py:148–151` `cyl(mnt_hole_d, 15.0, (-20.0, y, z))` → 刀 x 从 `H02_ARM_X[0]−3.0=−21.15` 到 `H02_ARM_X[1]+1.0=−11.0`（长 10.15、心 −16.075，从 H02_ARM_X 推，穿透 1.0） | 沿孔心线及 ±0.6 偏置 x 射线：臂端 ±0.6 内实体 0.500（x −12.5..−12.0） | 0.000（臂端 ±0.6 与整段孔 −18.65..−11.4 内均无实体）；`H02 arm seats` 6.1 不变 |
| R3g MAJOR | H01 右前脚承压圈起子槽 Ø4.3 对 60° 那条 Ø4.2 刀只余 0.048 | **修** | `duckstructure/head.py:43` `H01_FOOT_TOOL_D = 4.2 + 2*KO_OVER`（4.3）→ `4.5`；`tools/cad/mechanical_audit.py:365–372` N03 tool corridors 判据 ∩≤0.05 → 另加 H01/H02 `min_gap ≥ 0.15·cos(π/64)−1e-4`（yrm 仍判 ∩：刀端面按定义贴 A 盘坐面，gap 恒 0.01） | 刀↔H01 min_gap 0.048（process=True 网格 0.050），其余 5 条 1.94..2.00；孔壁到削面 1.310 | 0.1498（= 0.15·cos(π/64)），其余 5 条不变；孔壁到削面 **1.210**（≥1.2，review_r1 e 仍 PASS） |
| R3g MAJOR | L01 偏航端毂 Ø14.85 上 6 个 Ø2.4 过孔外缘到毂外圆 0.975（<1.2）；lib.py:45 注释 1.075 已过时 | **豁免** | `duckstructure/lib.py:45–48` 注释 1.075 → 0.975 + 豁免理由；`tools/cad/mechanical_audit.py:_review_r3 c` 新增断言 径向射线韧带 ≥0.9（左右件、x 13.3..16.8 八站位 × 6 孔） | 0.968..0.975（左右同） | 0.968（未改几何）。理由：毂 OD 受 6702 内径 15 定死、Ø2.4 用户定案（TC02 缩水）、分度圆 5.25 手册，三者不动；毂 x 13.0..17.0 整段就是内圈区（17.0..17.5 是 Ø4.4 沉窝层），没有"轴承不套的 x 段"可加厚；韧带非承力：螺丝预紧后过孔壁不受拉，径向力走毂外圆→内圈（受压）；打印后过孔缩 0.15/边 → 实际 ≈1.1。阈值 0.9 = 0.975 − 0.075 隙配裕量 |
| R3a MAJOR | T03 颈部区皮层被 N01 上排螺丝头扫掠（+NECK_CLR 0.8）削到 0.70（≈48 mm²） | **修**（降刀余量，不加厚壳皮） | `duckstructure/lib.py:91–95` 新增 `NECK_HEAD_CLR = 0.5`；`duckstructure/trunk.py:146–163` `neck_swing_env` 把 6 颗 `neck_screw_heads()` 从 base 里拆出、单独 `minkowski_box(±0.5)` 再与本体包络（仍 0.8）并起来 | 平面区皮层中位 0.70（min 0.68，169 点）；(17.5,·,153.9..155.4) 0.70；全区 <0.8 191 点≈48 mm²；头↔T03 零位/扫掠 0.800 | 平面区 **1.00**（117 点，min 0.988）；(17.5,·,153.9..155.4) 1.00；全区 <0.8 12 点（全在壳开口沿/底边圆角，削面从圆角穿出的"羽化"= 复审 NOTE，修前该类 19 点 min 0.05 → 19 点 min 0.35）；头↔T03 零位 0.500、扫 −60..45 每 1.25° 最小 0.500（≥0.3，壳因毂短一层高内移 0.2 后仍 0.3）。不需要局部加厚：加厚只会在开口沿圆角处造出台阶 |
| R3a MAJOR | L07 踝后臂定位榫槽与 M2 过孔之间 0.60 壁 | **修**（挪孔） | `tools/cad/ankle_split.py:9–14` `bolt_centers_yz` (±3, 18) → **(±3.4, 18)**（L05 Ø1.6 底孔 / L07 Ø2.4 过孔 / Ø4.4 头窝 / Ø6.4 凸台 / L06 起子孔全从它推，不涉及 L04）；`:52–55` 凸台在桥以外那段（主脚 x −10..−4.8、后臂 −22.5..−18）不再互相重叠 → 补 y ±1.6 腹板 | 槽壁→孔壁 0.60（x −14.8/−15.5，z 18，左右同）；头窝层 x=−20 壁 1.0 / 腹板 1.6 | **1.00**（左右同）；孔外壁 4.6 到桥边 6.2 = 1.6；头窝层 壁 1.0 / 腹板 2.4；L05↔L04 0.564/0.562@26.25、L07↔L04 0.400/0.353@8.75 与 r2 完全相同（凸台外移 0.4 不是最近点）；L06 与 L05/L07 ∩ 0 |

### 检查脚本
- `tools/cad/mechanical_audit.py:221–` 新增 `_review_r3()`（a 臂孔贯通 / c L01 毂韧带 ≥0.9 豁免 / d T03 头扫掠带平面区皮层 ≥1.0（−0.05）+ N01 头↔T03 扫掠 ≥0.3 / e L07 榫槽↔孔壁 ≥1.0），`:504` 挂进 verify_mechanics，`:545–` 汇总行 `[09-14 复审 r3 不变量]`；b 在 N03 tool corridors 里（`:365–372`）。反例：对 r2 的 placed/ 干跑 → a/d/e FAIL、c PASS（scratchpad/r3_audit_dry.py）。
- 复量脚本 `r3_measure.py` / `r3_n03tools.py` 正文附在 checks 日志末尾。

### Gate 数据（本次未改 yaml；待 Gate agent 跟）
- `features.yaml:1240` src_note "1.075（5.25）" 是旧 Ø2.2 算的 → 0.975；`waivers.yaml` 应加 L01 毂孔圈壁 0.975 的豁免条目（理由同上，非承力、受压）。
- `features.yaml:4649/4748/4891/5195`、`fasteners.yaml:5295` 的 (±3,18) → (±3.4,18)；L05/L07 新增凸台间腹板（主脚 x −10..−4.8、后臂 −22.5..−18，y ±1.6，z 16..21.2）。
- H01 脚 4 起子槽 Ø4.3 → Ø4.5（features 若有该特征）；H02 臂孔刀长 15 → 10.15（x −21.15..−11.0）；T03 颈部让位：螺丝头包络 0.8 → 0.5（本体仍 0.8）。

### 未做 / 观察
- 复审 MINOR/NOTE 未动；冻结件未动；手册值未动。T03 壳开口沿/底边圆角处削面"羽化"（NOTE）仍在：12 点 <0.8（修前 191，平面区那些已全部 ≥0.99），min 0.35@(21.5,139.5)。
- 三只读检查与 r2 一致（见 checks 日志）。

## 2026-09-14 Gate 数据同步到 r3 件（主线程）

对象 = `cad/duck_s288/*.stl` + `placed/`（build 日志 `docs/build_logs/build_2026-09-14_manual_dims_r3.log`，EXIT=2 仅 B01）。
只动 `tools/gate/data/*.yaml` 与 `tools/gate/retarget_horn_r.py`；**没动任何 Gate 检查器代码**（`layers/*.py`、`gate.py`、`core.py`）。
量法（全部只读、逐件加载、用完 `del`+`gc`）：
· **截面法**（scratchpad `survey_ring.py`）：沿孔轴每隔若干 mm 取 `mesh.section` → `to_planar` → 内环顶点 Kasa 最小二乘拟合圆 → 逐孔圆心/半径 → 再对 6 个圆心拟合分度圆，同时给相邻角。
· **射线/contains 法**（scratchpad `probe_line.py`）：沿一条直线 ≤180 点 `mesh.contains`，报料/空转变点。

### 1. 分度圆 4.75 → 5.25

**先修 `tools/gate/retarget_horn_r.py`**（唯一允许改的工具）：旧版 `RING` 正则只认写成一行的 `[[…]]`，而 `features.yaml` 33 处 `hole_positions_mm`、`fasteners.yaml` 13 处 `holes` **全是 YAML 块式**（`- - 4.75` / `  - 0.0` / `  - 15.35` 三行一点），所以 `--old 4.75 --new 5.25` 干跑只找到 12 处 `pitch_r_mm`、**一个孔环都没有**；照那样 apply 会让分度圆声明 5.25 而坐标还是 4.75。新增 `BLOCK_KEY` / `parse_block()` / `rewrite_block()`：缩进感知地解析块式点列，**只重写数值变了的那一行**（常量轴那行、`src_note` 等文字一律不动）。

`--apply` 实改 **38 处**：13 个 6 孔环（features 12 处 + fasteners 13 处里的同名环，见下）+ 12 处 `pitch_r_mm`。**13 个环逐个在新 STL 上实测，全部 r = 5.2500、相邻角全 60.000°、逐孔半径 min=max**：

| features.yaml 行 | 特征 | 件 / 轴 | 实测（截面法） |
|---|---|---|---|
| 1348 / 1371 | L01-F04 | L01_yaw2roll.stl / z | z −2.696..−0.273 共 4 截面，Ø2.3986..2.3991 → **r 5.2500** |
| 1442 / 1465 | L01-F05 | L01 / z | z 1.343 / 2.150，Ø4.3975/4.3980 → **r 5.2500** |
| 2426 + 2468 / 2388 | L02-F06（两片法兰） | L02_hip.stl / z 与 x | z −3.128..3.290 共 9 截面 + x 17.523..24.499 共 9 截面 → **r 5.2500** |
| 3652 / 3675 | L04-F08 | L04_lower_leg.stl / z | z −0.341..3.772 共 14 截面 → **r 5.2500** |
| 4260 / 4283 | L05-F04 | L05_ankle_foot.stl / z | z −1.170..3.807 共 7 截面 → **r 5.2500** |
| 4353 / 4376 | L05-F05 | L05 / z | z 4.636，Ø4.3974 → **r 5.2500** |
| 6191 / 6214 | T01-F08 | T01_trunk.stl / y | y 14.222 / 15.459 → **r 5.2500** |
| 7923 / 7946 | T03-F06 | T03_shell_R.stl / y | y −18.003..−13.465 共 8 截面，Ø2.5985..2.5996（惰轮侧）→ **r 5.2500** |
| 9341 / 9364 | N02-F07 | N02_neck_pitch.stl / z | z −1.286..1.571 共 5 截面 → **r 5.2500** |
| 9439 / 9462 | N02-F08 | N02 / y | y 21.703..25.192 共 6 截面 → **r 5.2500** |
| 9626 / 9650 | N02-F11 | N02 / z | z −30.571..−27.714 共 5 截面（Ø2.6）→ **r 5.2500** |
| 10042 / 10065 | N03-F04 | N03_yaw_roll.stl / x | x −17.537..−14.788 共 5 截面，相位 30° → **r 5.2500** |

`fasteners.yaml` 里 13 个 `feature_hole_map[].holes` 是同一批坐标的副本（F02 L01-F04/F05、F04 L02-F06 inst0、F06 L02-F06 inst1、F08 L04-F08、F10 L05-F04/F05、F12 T01-F08、F12b T03-F06、F14 N02-F07、F14b N02-F11、F16 N02-F08、F17 N03-F04），一并缩放。
12 处 `pitch_r_mm` 的 `src_note` **保留旧文字**、后面追加日期 + 手册/实物依据 + 本次实测数（`src` 改 `measured`）。`tolerances.yaml:fits.horn.clock_phase.nominal_deg.src_note` 的 "6 孔 @ r4.75" → r5.25。

**探针产物一律没缩放**：`fasteners.yaml` 25 个紧固件组各加一行 `stale_since: '2026-09-14 (horn_r 5.25 + r1/r2/r3 改件，待重跑探针) …'`，明确覆盖 `head_locator_map_indices` / `tool_access[].seats[]` / `tool_access_probe_2026-09-13` 三个键，值全部保持原样。全库残留的 122 行旧环坐标（4.1136 / 2.375 / ±4.75）**全部且仅在** `tool_access[].seats[].point_export_local` 下（脚本核对过父键），正是应该留的那些。

### 2. r1/r2/r3 改件的声明同步（每条在新 STL 上实测后写数）

| file:line | 键 | 旧 → 新 | 依据 |
|---|---|---|---|
| features.yaml:1240 | L01-F02 `nominal_d_mm.src_note` | "1.575(4.75)/1.075(5.25)" → 补注"这两个数按旧 Ø2.2 算、已过时；Ø2.4+5.25 → **0.975**，r3 径向射线实测 **0.968**" | CHANGELOG r3 第 3 条 |
| features.yaml:1549 L01-F07 / 2507 L02-F07(×3) / 3565 L04-F07 / 4403 L05-F06 / 9496 N02-F09 | `nominal_d_mm` | 14.3 → **14.6**；span/pos 各 +0.15 | lib.py:430 `flange_relief` 默认 c 0.2→CLR 0.3；截面实测 L04 Ø14.5944 @z−3.504、L02 Ø14.5993 @z−3.500 |
| features.yaml:4316 L05-F05 | `depth_mm` / span 起点 / spec | 1.2 → **1.05**；3.85 → **4.0**；`horn_cbore(Ra,19.55,1.2)` → `(19.7,1.05)` | legs.py:162 沉窝起点 19.55→19.7；Gate L2 逐孔实测掏空量 6/6 全 **1.05** |
| features.yaml:4643 L05-F10 / 4742 L05-F12 / 4885 L06-F03 / 5189 L07-F04 / 5246 L07-F05 / 5312 L07-F06 | `hole_positions_mm` 的 x | ±3.0 → **±3.4**（12 个点）+ 4 条 `spec_verbatim` | ankle_split.py:9-14；截面实测 L05 Ø1.6 底孔心 (±3.4000, −18.0000)、L07 Ø2.4 过孔心 (±3.4000, −18.0000) |
| fasteners.yaml:5319 | F24 `location` | `bolt_centers (±3,18)` → `(±3.4,18)` | 同上 |
| features.yaml:8272/8351/8416/8461 N01-F01..F04 | `bbox_mm` z / `pos` z / `rear_boss_relief.axial_span_mm` | 27.65→**27.8**、30.65→**30.8**、29.15→**29.3**、28.65→**28.8**（共 15 个数）+ 3 条 spec_verbatim 的 y −16.15..−13.15 → **−16.3..−13.3** | 背板 3.0 站位随法兰面 12.85→13.0；N01 自己的 frame_note 写死 `export_z = 14.5 − x_servo`；修前 L2 三条 `min_section` 全报 2.85 = 旧窗与新板的交集长度 |
| features.yaml:8850 N01-F10 | `nominal_d_mm` / `depth_mm` / span / spec | Ø15.0 → **16.0**；刀长 12.15 → **12.0**；span 27.35 → **27.5** | neck.py:20 `NK_IDLER_BORE` 15→16；截面实测 z 27.886..30.799 十三个截面 Ø15.9889..16.0000 |
| features.yaml:9240 N02-F06 / 9664 N02-F12 | `nominal_d_mm` + spec | Ø13.8/14.0 → **15.0** | `NP_CB_D = 2·(horn_r 5.25 + 4.4/2) + 0.1`；截面实测 z 2.406/2.999 → Ø14.9910/15.0000，z −31.999/−31.406 → Ø15.0000/14.9910 |
| features.yaml:9708 N02-F13 | `nominal_d_mm` / `depth_mm` / span / pos | Ø14.0 → **15.0**；高 1.65 → **1.5**；span 端 −27.35 → **−27.5**；pos −28.175 → **−28.25** | neck.py:105 `NP_IDL_HUB_D` 14→15；惰轮端面 −12.85 → −13.0；L2 24 向射线实测 r 7.492..7.500 |
| features.yaml:7787 T03-F04 | `nominal_d_mm` / `depth_mm` / span / pos | Ø14.0 → **15.0**；高 5.55 → **5.4**；span 端 −12.85 → **−13.0**；pos −15.625 → **−15.7** | trunk.py:164 `SHR_IDL` 14→15；同上端面 |
| features.yaml:11767 H02-F06 | `depth_mm` / span / positions / spec | 刀长 15.0 → **10.15**；span z (35.6,20.6) → **(29.25,19.1)**；刀心 z 28.1 → **24.175** | head.py:148-151 刀 x −21.15..−11.0（r3 BLOCKER：旧刀只到 −12.5，臂端 −12.0 留 0.5 膜）；`probe_line` 实测孔心线 z 17..38 全空（真通），旁边 y=14.6 的臂料 z **20.0..26.4** 被新刀完整跨过 |
| features.yaml:10100 N03-F05 | `depth_mm` / span / pos / spec | 刀长 3.6 → **3.25**；span x (−14.8,−11.2) → **(−14.45,−11.2)**；pos −13.0 → **−12.825** | neck.py:150 `YRM_BORE=(YRM_A2,14.8)`，YRM_A2 11.2→11.35→**11.55**；`probe_line` 在 r=7.0/7.9 四个方位实测 料→空 转变全在 x = **−14.475 ± 0.025** |
| features.yaml:7427/7610（T02/T03 颈摆包络）| `spec_verbatim` / `derived_from` | `minkowski 0.8` → 0.8（本体）/ **0.5**（N01 上排 6 颗螺丝头） | r3 第 4 条：lib.py:91-95 `NECK_HEAD_CLR=0.5` |
| keepouts.yaml:428 KO10 | `geom_verbatim` | `外扩 0.8` → 0.8 / **0.5**（同上） | 同上 |
| features.yaml:992/1022（N03_Rr / H01_Rr 帧表）| `world_origin_mm` x / `what` | −4.9 → **−4.7**；"SERVO_SHIFT 后挪 1.5" → **1.7** | r1 M7：lib.py:137 `SERVO_SHIFT[('jaw_soft',1)]` −1.5→−1.7 |
| parts.yaml:576 | servo_head_roll 描述 | `-1.5 / (-4.9,0,235.61) / 法兰面 7.95` → **-1.7 / (-4.7,0,235.61) / 8.3** | 同上（7.95 还是 flange_h 2.9 时代的数） |
| relations.yaml:R11 | `actual` / `actual_src` / `status` / `note` / 两个 `face` | 旧"壳座 r10.89 → 0.09/0.29" → **H03/H01 A 半座 r 11.085..11.088 = Ø22.170..22.176、N03 discA r 10.787..10.800 → 径向 0.285**（零位 0.2847、±25° 最小 0.2717@+14°）；status open → `resolved_2026-09-14` | r2 M5 复量（本次未重跑 contact_profile） |
| relations.yaml:R11.contact | 新增 `stale_since` | — | `measured_2026-09-09` 那组是 r2 改件之前量的，本次没重跑，明确标死 |

### 3. 两条豁免（`waivers.yaml`，本文件此前是空的 `waivers: []`）
- **`L2/L01:L01-F04:wall`** —— L01 偏航端毂 6 个 Ø2.4 过孔的径向韧带 **0.968**（名义 0.975 = 14.85/2 − (5.25+1.2)，判据 1.2）。理由：过孔非攻牙孔、预紧后孔壁不受拉；6702 内孔 Ø15 对毂 Ø14.85 是 0.075 **间隙配合不是压装**；扭矩剪切路径走周向腹板 3.10、不过这道径向韧带；三个上游尺寸（6702 内径 / Ø2.4 过孔 / 分度圆 5.25）都不许动，毂 x 13.0..17.0 整段是内圈区没处加厚；打印后缩 0.15/边 → 实物 ≈1.1。**另附**沉窝层（x 17.0..17.5）Ø4.4 头窝外缘 r 7.45 > 毂 OD 7.4125，6 处各破出 **0.025 深 × 0.82 宽**的豁口 —— 在轴承座区之外、M2 头外缘 7.25 仍全落在毂上，非承力。
- **`L5/H01:foot_tool_slot_d4p5_clearance`** —— H01 右前脚起子槽 Ø4.5：真圆起子余量 **0.1473**（判据 0.15，差 0.0027，因为判据是"64 边刀对同相位 64 边槽"，网格值 0.149817 = 0.15·cos(π/64)）＋孔壁 **1.2034**（判据 1.2，网格值 1.210）。两个数是同一把刀的两端，Ø4.3 时起子余量只剩 0.048，Ø4.5 时两条判据把它夹在**唯一交点**上 —— 豁免里写死 **"Ø4.5 与分度圆站位都不许再动"**。该条带 `gate_cell_note`：features.yaml 里还没有这 6 条槽的独立特征，判据现在只活在 `mechanical_audit.py:365-372`，等特征补出来后把 `cell_id` 指过去。
两条都按 schema 绑了 `bound_input_hashes`（各 5–6 个键，含该件 STL、相关 `duckstructure/*.py`、features/tolerances），**没绑 `waivers.yaml` 自己**（它也在 source_manifest 里，绑了是死循环）。

### 4. 子集验证 `./.venv/bin/python tools/gate/gate.py --layers 0,2`

日志 `docs/build_logs/gate_2026-09-14_r3_L0L2.log`；记分卡 `tools/gate/out/scorecard.partial.{md,json}`。
反例 64/64 全过（219 s）；L0 96 条判据 / 16 红；L0+L2 累计 611 条 / 189 红；判定 `BLOCKED（子集跑）✅16 ❌28 🕓427`。
记分卡里 layer∈{0,2} 的判据共 **711 条：FAIL 205（BLOCK 87 / WARN 118）、WAIVED 1**。

**与 2026-09-13 全量记分卡（`tools/gate/out/scorecard.json`，旧件旧数据）逐判据对照：由红转非红 61 条，新红只剩 4 条。**
本轮同步一共消掉的新红 = 22 → 4，消掉的 18 条全部是"声明没跟上件"，已改完（见第 1、2 节）：
L01-F07:present、L02-F07:through、L04-F07:present/through、L05-F05:depth、L05-F12:present/bore_gauge、
L07-F05:through、N01-F01/F02/F03:min_section、N01-F10:present/through/bore_gauge、N02-F06:through、
N02-F12:through、N02-F13:present、T03-F04:present；外加本轮顺手修好的 09-13 遗留红
H02-F06:present/through/bore_gauge（r3 BLOCKER 的 0.5 膜，刀长改 10.15 后 16/16 向真通）和 N03-F05:through（8/8）。

剩下 4 条新红，**都不是声明问题，一条没改**：
| 判据 | 实测 | 归类 | 说明 |
|---|---|---|---|
| L0/L03 `degenerate_faces` = 6 / `nonmanifold_edges` = 5 | — | **件的问题** | L03 导出网格质量；洞边 0，只有 L04 有严格断言。与本次数据同步无关，属 build 侧 |
| L2/L06 `L06-F03:wall` = 0.8671（09-13 为 1.2879） | 0.8671 | **判据问题 +件的变化** | 螺栓中心 ±3→±3.4 后 TPU 鞋底让位孔离边更近。阈值本身是"TPU 95A 结构墙最小厚度"被借来当孔周壁厚（判据自己写了 `status=unchecked_by_gate` → 记 WARN）。L06-F03 的 present/through/bore_gauge 09-13 起就一直红，原因写在该特征 `verified_on_export.note`：Ø2.6 整个落在 L05/L07 的 Ø6.4 凸台掏出来的空腔里，在成品 L06 上量不到独立的孔 |
| L2/N03 `N03-F05:present` = 20/24 向命中（4 个方位 r=8.2） | r 7.987..8.2 | **件的问题 / 判据与特征定义矛盾** | 该特征 purpose 原文就是"让刀 **+ 铲掉原版 XL330 4 孔**"，那 4 个方位量到的 8.2 正是被铲掉的 4 个孔留下的缺口，判据却要求 24/24 量到整圈 Ø16。09-13 能过是因为旧站位下 21 个探测站正好落在纯圆柱段。同一条特征的 `through` 本轮已由红转绿（8/8 真通）|

豁免落地情况：`L2/L01:L01-F04:wall` → **WAIVED**（绑定 hash 全对上）；`L5/H01:foot_tool_slot_d4p5_clearance` 没有对应格子，Gate 直接忽略（`waiver_notes` 为空，不影响判定），等 H01 脚起子槽在 features.yaml 里补出特征后再指过去。

### 5. 明确没同步的
- `fasteners.yaml` 25 个组的 `head_locator_map_indices` / `tool_access[].seats[]` / `tool_access_probe_2026-09-13`：按分工**一律不许缩放**，只加了 `stale_since`。要更新必须重跑 `tools/gate/probe_seats.py`。组号：F01/F02/F03/F04/F05/F06/F07/F08/F09/F10/F11/F11b/F12/F12b/F13/F14/F14b/F15/F16/F17/F18/F19/F21/F22/F24。
- `features.yaml` 里**还没有的新特征**（本次只同步已有条目的值，没有新增条目）：H01 右前脚 6 条 Ø4.5 起子槽、T02 轭区开口（SHL_YOKE_*，r2 M1 把原版 lobe 整块开掉）、两壳的髋偏航惰轮 Ø14.8 让位柱（r1 M5/M6）、H03 的 H02 让位（r1 M7 连带）、L04 电池让位由名义实体改仓腔扫掠（r2 M6）、L05/L07 凸台间新增腹板（r3 第 5 条）、N03 新增的 `flange_relief(Rr)`（r1 M7 连带；截面在 N03 x −19.6..−17.83 量到 Ø14.5946..14.6028）。
- `relations.yaml:R11.contact.measured_2026-09-09` 那组 signed_* 数：没重跑 `l5_function.contact_profile`，只加了 `stale_since` 并把结论改写到 `actual`。
- `tolerances.yaml:fits.horn.clock_phase.applies_to_features` 只列了 8 条（缺 L01-F05 / L05-F05 / T03-F06 / N02-F11 这 4 条同样带 6 孔环的），本次没动 —— 那是判据覆盖面问题，不是分度圆同步。
- `assembly.yaml`：T02 装配序对 F12 的备注（r1 待跟项）没改，因为 r2 M1 已经把 r1 的 6×Ø4.6 通孔方案整个换成轭区开口，等 T02 的新特征条目补出来再一起写。
- `l1_calibration.yaml` / `printability.yaml` / `bench.yaml` / `components.yaml` / `harness.yaml` / `frozen.yaml`：本次一个字没动（frozen 的 `SERVO_SHIFT jaw_soft_1 −1.7` 在 r1 已改完）。

### 2026-09-14 装配序文案与螺丝规格对齐（主线程）
- `assembly.yaml:728` step 8 action：`6×M2×12` ×3 处 → `6×M2×10`。依据 `fasteners.yaml` F04/F06/F08 的 `spec=M2×10`（叠厚 7.5 + 咬入 2.5 = 10.0，三组一致）。原文 M2×12 是旧规格残留。
- `assembly.yaml:1389` step 10 action：`6×M2×8（F02）` → `6×M2×6（F02）`。依据 09-14 复审 r1 把 F02 由 M2×8 改 M2×6（叠厚 5.35→4.0，头顶不再穿横滚舵机顶端），`fasteners.yaml:F02_hipyaw_flange.spec=M2×6` / stack 4.0 / engagement 2.0。
- 只改 action 文案（人按它备料和装配），未动 `fasteners` 字段与判据。发现途径：用户问 L01 与两颗舵机的连接顺序时逐条核对。

### 2026-09-15 打印朝向试验 → L01/H02 改朝向（主线程）
- `printability.yaml:parts.L01.down_world` [0,0,-1] → **[-1,0,0]**（横滚轴竖直、6704 座口朝上）：F10 座内落点 38922→0，禁撑区总落点 51180→12159，首层 11.6→215.2 mm²。
- `printability.yaml:parts.H02.down_world` [-1,0,0] → **[1,0,0]**：F07 锪平面落点 2440→0；支撑 94→234 mm³。
- 两件已重切进 `tools/gate/slicing/slice_run.yaml`；全表与 H01/L03 结论见 `docs/reports/打印朝向试验_2026-09-15.md`。
- 试打件 `V2_S288版发布产物/09_试打件_2026-09-14/L01_yaw2roll_试打_v2_座口朝上.stl` 按新朝向导出；09-14 上午那版 +y 朝向错误已删除。

### 2026-09-15 L6 目标区间收窄 + F03 可达性（主线程）
- `frozen.yaml:joint_axes[].target_range_deg`（6 个关节）：按 r3 件 L6 单轴实测无碰撞区间在**碰撞端**收 3°（无碰撞端与上游重合、保持上游值）：left_knee [−90,90]→**[−90,62]**（无碰撞 [−90,65]）；left_ankle→**[−62,62]**（[−65,65]）；neck_pitch [−90,60]→**[−62,44.5]**（[−65,47.5]）；head_pitch→**[−62,90]**（[−65,90]）；right_knee→**[−62,90]**（[−65,90]）；right_ankle→**[−62,62]**（[−65,65]）。每条新增 `date: "2026-09-15"` + `reason`（无碰撞区间、哪端收、重训、上游值保留），`src` 改为 L6 实测出处，旧值与旧 src 原文挪进 `src_note` 开头；`range_deg`（上游值）未动。其余 8 个 MJCF 关节单轴全程 0 条碰撞，target 保持上游值；home ±10° 都在无碰撞区间内，无"需要改件"的关节。依据：用户 2026-09-14 决定（重训、限位按自己的件定）。
- L6 前后（`gate.py --layers 6`，r3 件，64 反例全过）：判据 FAIL(BLOCK) **54 → 43**、FAIL(WARN) 4 → 14、PASS 16 → 17；碰撞记录目标区间内 16328 → 2111。转 WARN 的 11 条 = 6 条 single_axis_sweep + 2 条 pair_combination（hip_pitch+knee 左右）+ 3 条 motion_collision（N01/N03/N04）；`_ranges/gen_range_covers_target` 转 PASS。剩下的 43 条 BLOCK：组合姿态碰撞（hip_yaw×hip_roll 内收+内偏航撞电池仓/门 607.7、hip_roll×hip_pitch 外展+前抬撞壳 1232.4、knee×ankle 膝 50..60 × 踝 −60..−30 撞大腿舵机 607.0、neck×head 颈 +5..40 × 头 −60..−25 撞躯干/电池 1553.2、head_pitch×head_yaw −60..−50 × ±140..170 撞壳 278.2、head_yaw×head_roll 全盒蹭颈俯仰件 56.2）及其落到 14 个件的 motion_collision / KO10 / KO13 / combo_collision_inside_free_range / collision_buckets；另 12 条是线束无模型（HB01..09、KO19）、mouth 无判据、阈值借用、min_clearance、旧策略包络、生成 MJCF 缺 foot geom。逐条与步态影响见 `docs/reports/关节限位收窄_2026-09-15.md`；日志 `docs/build_logs/gate_2026-09-15_L6_{before,after}.log`。
- `negatives/n_l6_target_range_severity.py`：B 组"目标区间 = 上游值"原来直接拿真 frozen.yaml，收窄后 left_ankle 真值已是 [−62,62] → B 组 ≥70° 碰撞落到区间外，反例失败（第一次 after 跑 `64 个反例，有失败`，日志弃用）。改成 B 组显式 `v = range_deg`（上游拷贝）、去掉 date/reason；反例语义不变、不再依赖真数据是否收窄。层代码（`layers/*.py`）未动。
- F03 髋横滚背可达性（r3 placed STL，seq 9 工位；`docs/reports/F03可达性_2026-09-15.md`）：坐面重找于世界 x −7.29（09-13 −7.24）；外排 y=±25.5 Ø4.2 直杆到机外 37.5/44.5 mm 交集 0；内排 y=±9.5 从坐面 −x **13.81 mm**（头顶起 12.2）撞 T01 电池仓前壁（x −21.2..−24.6 厚 3.4，壁 |y|≤14.5；Ø4.2 交集 48.5 mm³、Ø4.0 44.0），壁后 18.0 mm 起是电池。① 短批头：直杆/任何 1/4" 批头+手拧套不成立（≥20 mm > 12.2）；只有弯头/L 形 PH0 短脚 ≤11 mm 成立（长脚朝 +y 外侧或 −z 下方，口袋 x −9..−20 截面 y 9.5..45、z 70..118 全空）。② 改下排 z=80：L01 下缘 z=87 无料，且内排仍正对同一堵壁（舵机背面起 14.2 mm）→ 不成立。③ 改序先拧 F03 再上 T01：F02 十二个坐面（z 114.5，头朝 −z）被横滚舵机顶面挡在 2.5 mm（头顶余 0.9）→ 不成立。
- `assembly.yaml`：step 6（seq 9）与 step 10（seq 8）各加 `note_2026-09-15`（只加注释字段，seq/action/motions/tool_states 未动）。`fasteners.yaml:F03` 未改（seats 仍标 stale，本次重找只差 0.05 mm；正式更新要重跑 `probe_seats.py`）。`features.yaml` 未碰（另一 agent 在改）。

## 2026-09-15 新几何特征声明补齐（主线程）

对象 = `cad/duck_s288/*.stl`（r3 build，未重建）。只动 `tools/gate/data/features.yaml`（改前副本留在 scratchpad `features.yaml.bak_2026-09-15`，会被清理）、`parts.yaml` 的 feature_count / features_total、`waivers.yaml` 两条豁免的 features.yaml sha 重绑（见 §子集验证 末尾）；**没动任何 Gate 检查器代码、没动 frames**。
起因：09-14 r1/r2/r3 在件上新增的几何在 `features.yaml` 里没有条目（09-14 数据同步节 §5 列的那份清单）——Gate 只查声明过的东西，没声明就等于没查。
量法（全部只读、逐件加载、每次 `contains` ≤200 点、采样 ≤20000 点；脚本 scratchpad `nf_probe.py` / `nf_sweepgap.py` / `nf_insert.py` / `nf_modify.py`）：
· **射线**：从声明轴心按站位打 24/48/72 向，收窗内全部穿越半径；轴向射线量端面/刀底。
· **contains 线/网格**：沿一条线 ≤200 点报料/空转变；在声明盒内撒网格数有料点。
· **扫掠间隙**：动件顶点 + 8000 表面点按世界系绕关节轴转 N 个角 → KD 树预筛 → `trimesh.proximity.closest_point` 到静件（仓腔用解析盒距离）。
声明策略（一句话）：刀在成品上只留下一小段弧面（起子槽 / 惰轮让位柱 / 挡肩刀）或非解析（轭区开口、H02 包络）的，**不给顶层 `nominal_d_mm`**（24 向整圈判据对它不成立），改用 `shape: box` + 刀的**内接方柱**按「挖除类：区域内无料」判，刀本身写在 `cutter:` 子键（沿用 N03-F08 `cylinder:` 子键的先例）；能量到整圈的（N03 法兰让位）照常给 `nominal_d_mm` 走射线判据。

### 新增 10 条（id / 源码 / 实测）
| id | kind | 源码 file:line | 实测（新 STL） |
|---|---|---|---|
| **L01-F15** | clearance_cut | lib.py:560-563 `ring_boss` 挡肩刀 `cyl(lip_d=24, x xf−1.0..xf+0.05)`; legs.py:39-43 | 刀起面 export y **12.00**（r 9.5/11 十三个有料方位沿 y 逐点，y<12.0 料、≥12.02 空）；盘 r≤11.9 × y 12.05..12.85 **0/1080** 点有料；Ø24 刀痕 2/72 方位 r **11.9857**（=12·cos π/64）；挡肩环 r 13.3 在 y 12.2/12.6/12.85 有料 36/46/50 方位 |
| **N03-F10** | flange_relief | neck.py:221 `flange_relief(Rr)`; lib.py:442-447 | 24 向 × 6 站（x −19.5..−18.0）**144/144** 命中 r 7.2922..7.3046 → **Ø14.584..14.609**；坑底 x **−17.683**（法兰面 −17.7）、A 盘端面 **−19.593** → 件上盲坑深 **1.9**（刀 3.3 里 1.4 在件外） |
| **H01-F15** | tool_channel ×6 | head.py:43 `H01_FOOT_TOOL_D=4.5`; head.py:112-115 | 6 刀各 864 点（r≤2.24 × z −45..−37）**全 0** 有料；60° 刀在 z −42/−41/−40 三站 0°..37.5° 六向命中 r **2.2476..2.2500**（64 边内接 2.2473），其余 5 刀无任何命中；Ø2.7 孔壁→弧面最薄 **1.2034** @(x −2.625, z −41.0)（= waivers.yaml L5/H01 / review_r1 e 的 1.2034） |
| **T02-F04** | window | trunk.py:188-192 `SHL_YOKE_*`; trunk.py:238-243 | 弧 r **8.8159..8.8176**（128 边内接 8.8173）在 10°..30°（左下）三站 y 16.7/17.2/17.7；直边 x = **21.0000**（−x 射线 z 26/28/36/38 首命中；40°/115°..145° 射线 r·sinθ = 5.000）；180° 头通道远壁 x **18.4501**；壁 y **16.6..17.9**；上缘 z 39.5 有料 / 40.5 空；三个内接盒 0/125 × 3 有料 |
| **T02-F05** | clearance_cut | trunk.py:177 `HIP_IDLER_RELIEF_EXTRA`; trunk.py:222-224 | Ø14.7 × z 21.55..24.75 网格 **0/1512** 有料；筋切面 r **7.3938..7.4000**（305°..325°，z 23.5..24.8）；刀端面 z **24.800**（+z 射线首命中）；筋 z 起点 23.05；刀外 z≥25.1 筋内缘 r 6.976..7.173（修前 6.97） |
| **T03-F07** | clearance_cut | 同上 | 网格 **0/1512**；筋切面 r **7.3965..7.4000**（220°..225°）；刀端面 z **24.800**；筋 z 起点 23.13；刀外筋内缘 r **7.3428..7.377**（修前 7.339，刀只削 0.06） |
| **H03-F07** | clearance_cut | head.py:165 `clamp_clr = minkowski_box(build_head_clamp(), ±CLR)`; :182 diff | min_gap(H02→H03) **0.3000** @ export (−10.384, 7.534, 22.8) = 下垫柱 +x 端面(世界 x −14.7) 对原版地板垫台；H03→H02 **0.3000**；H02 包围盒中心在 H03 内 contains=False |
| **L05-F13** | web | ankle_split.py:54 `main += b((-10,-1.6,16),(-4.8,1.6,21.2))` | 15 条沿 x contains 线（y −21..−16.2 × z −24.3..−19.5）料段全部包住 [−1.6,1.6]（y −21: −4.40..2.11；−18.6: −2.83..2.96）；x=0 沿 z 料到 **−19.25**（盒顶 −19.3）；腹板嵌在原版脚体料里 |
| **L07-F09** | web | ankle_split.py:55 `arm += b((-22.5,-1.6,16),(-18,1.6,21.2))` | y −21 × z −36.8..−33.5 精确射线料段 (−1.600, 1.600) = **3.200**；头窝层 y −18.6 (−1.284, 1.284) = **2.568**（Ø4.4 头窝弦；名义 2.4@y −18.0）；x=0 沿 z 料 −36.95..−28.2 |
| **L07-F10** | sweep_cut | legs.py:172 `clr = sweep_of(ll,'ankle_left',±62,21,grow=0.4,mink=True)` | L04 绕踝轴 17 个角 vs L07：16 个 **0.4000**、+40° **0.395**（≥0.3 判据；<0.4 = 扫掠只取 21 姿态的采样效应） |

### 改 3 条（mink=True 与扫掠对象）
| id | 改动 | 实测 |
|---|---|---|
| **L04-F12** | spec/derived_from 加 `mink=True`（legs.py:147）；加 `grow_method`、`pos`/`box_export_local`（膝环座凸角 +y 方位 r 15.45..15.75 × z 0.55..0.9 那一小段） | L03 绕膝轴 −62..92 共 16 角 vs L04 min_gap **全 0.4000**；r ≤16.0 × z 0.55..0.9 全 72 方位无料，L04 板角料从 r 16.1 起 |
| **L04-F13** | 扫掠对象 电池名义盒 → **仓腔** `wbox(BX0..BX1, ±BAT_Y, BZ0..BZ1)` + `mink=True`（legs.py:148-153）；`swept_box_bbox_mm` 改成 45 姿态并集 **[−26.58,−66.4,3.94]..[20.29,6.8,72.40]**（旧值其实是零位盒，另存 `bay_bbox_zero_pose_export_mm`） | L04 顶点+20000 点 vs 仓腔盒（解析距离）−30..25 每 1°：最小 **0.4000 @ +25°**（sweep_of 角度约定；= CHANGELOG r2 M6 的『−25°』，关节角符号相反）、任何姿态在仓内 0 点；L04 体积 **5493.761** |
| **L05-F08** | 第 2 实例（clr）加 `grow_method: minkowski_box(±0.4)`、`pos`/`box_export_local`（分体面 r≤13 内一片）；spec 改现行 ±62/n125/n21 + mink | L04 绕踝轴 13 角 vs L05 min_gap **0.5616..0.5642**（±20° 最小）；盒内 7×5×3 网格 0/105 有料 |

### 只核不改
- **H02-F06**（09-14 同步 agent 已改 (29.25,19.1)）：两孔轴线 z 15..40 contains 全空（真通）；臂料 y 14.6 / 12.8 处 z **20.15..26.31** 被声明区间 19.1..29.25 完整跨过；Ø2.4 在 z 21.4/23.2/25.0 三站 24/24 向 r 1.1987..1.2000。**不改**。

### 计数
`features.yaml:counts` 172 → **182**（by_kind：clearance_cut 6→10、web 1→3、sweep_cut 11→12、window 2→3、flange_relief 5→6、tool_channel 4→5；by_part：L01 15、L05 13、L07 10、T02 5、T03 7、N03 10、H01 15、H03 6），`parts.yaml` 对应 8 件 feature_count 同步、`features_total` 171 → 182（171 是 09-12 旧值）。frames 未动。

### 子集验证 `./.venv/bin/python tools/gate/gate.py --layers 2 --parts H01,H02,H03,T02,T03,L01,L03,L04,L05,L07,N03 --out tools/gate/out/newfeat_2026-09-15`
（`--out` 另开目录：另一 agent 同时在跑 `--layers 6`，两个子集跑都写 `scorecard.partial.*`，不能共用 `out/`。）日志 `docs/build_logs/gate_2026-09-15_newfeatures_L2.log`。
反例 64/64 全过（223.2 s，features.yaml 变了所以缓存未命中）；L2 **390 条判据 / 120 红**（记分卡 `tools/gate/out/newfeat_2026-09-15/scorecard.partial.{md,json}`；总判定 BLOCKED 的 ✅2 ❌262 里 260 个是子集跑之外的层/件按义务记红，不是本轮量出来的）。
**本次新增 10 条 + 改 3 条 + 核 1 条，共 13 个 id 的判据全绿（0 红）**：
L01-F15 occupancy 28/28；N03-F10 present 24/24 · diameter Ø14.6000（棱边 552 条）· through 0/8 真通 · depth 1.9 · wall 3.4954；H01-F15 occupancy 66/66（参考点 6/6）；T02-F04 63/63（3/3）；T02-F05 28/28；T03-F07 28/28；H03-F07 1/1（sweep 只判参考点，WARN 级）；L05-F13 occupancy 28/28 · solid_retention 245/245=1.0 · min_section 3.2；L07-F09 occupancy 24/28（头窝层角点无料属声明的 L07-F06 头窝）· solid_retention 170/170=1.0（扣 40 格声明孔槽）· min_section 3.2；L07-F10 / L04-F12 / L04-F13 / L05-F08 occupancy 1/1（sweep，WARN 级；L04-F12 / L05-F08 由 unknown『geometry』转绿）；H02-F06 present 2/2 · diameter Ø2.4000 · through 16/16 · bore_gauge 2.3971（wall 1.3 <2.5 WARN 是 09-13 起就有的旧红）。
120 红的归类：7 条 `obligation_unmet` = 子集外的件（B01/L02/L06/N01/N02/N04/T01）按义务记红，不是量出来的；113 条全部落在**本次没动的旧条目**上（wall 43 / present 26 / through 11 / solid_retention 6 / bore_gauge 6 / min_section 5 / occupancy 4 / depth 4 / geometry 3 / diameter 3 / polarity 1 / 覆盖率 105/182 1），71 WARN / 49 BLOCK，与 09-14 L0L2 子集跑的红集合同类（判据问题：PLA min_wall 2.5 借作孔周壁厚的 wall 类；件的问题：N03-F05 present 20/24 等；声明问题：09-14 节 §4 已列），本次不重分类。
副作用：`waivers.yaml` 两条豁免都绑了 `features.yaml` 的 sha，本次改文件后自动失效（记分卡 waiver_notes：『L2/L01:L01-F04:wall 的豁免已失效（输入变了）』）→ 两条的 features.yaml sha **重绑**到新值，各加 `rebind_2026-09-15` 说明（豁免对象 L01-F04 声明未动；H01 那条注明特征现为 H01-F15、但 Gate 仍无可指格子）。重绑后只跑 L01 复核：`docs/build_logs/gate_2026-09-15_newfeatures_L2_L01_waiver_rerun.log`：反例 64/64（208.7 s），L2/L01 54 条判据 / 19 红 / **1 WAIVED = L01-F04:wall**（waiver_notes 空），L01-F15 occupancy 28/28 PASS；19 红全是 L01 的旧条目（同上表）。

### 未做 / 观察
- `waivers.yaml:L5/H01:foot_tool_slot_d4p5_clearance.gate_cell_note` 说等特征补出来后把 `cell_id` 指过去 —— 特征现在是 **H01-F15**，但 L2 对它只判「刀内无料」，起子真圆余量 0.1473 / 孔壁 1.2034 仍只活在 `mechanical_audit.py:365-372`，没有可指的 Gate 格子；只在 `rebind_2026-09-15` 里写明，cell_id 未改。
- L01-F10（6704 座 Ø27.15）声明的 x 12.75..16.95 是 xf 12.85 时代的数（现 12.9..17.1），09-14 同步没跟；本次不在清单，未动。
- L02 髋横滚扫掠（legs.py:41 `mink=True`）不在本次件清单，L02-? 的 sweep_cut 条目未加 mink 注记。
- `fasteners.yaml` 探针产物、`assembly.yaml` T02 装配序备注（等 T02-F04 出来再写）仍未动。

### 2026-09-15 壳柱螺丝坐面锪平（主线程，CAD 已改、未 build）
- `duckstructure/lib.py` 新增 `SHELL_SEAT_D/SHELL_SEAT_MARGIN/SHELL_SEAT_MIN_WALL` + `shell_seat_z()`；`duckstructure/trunk.py:build_trunk_shell` 壳柱孔后加 Ø5 口袋刀（平台 = 头足印最低外表面 −0.1）；`tools/cad/mechanical_audit.py:_review_r4`（平面度 ≤0.02 / 壁 ≥1.2 / 平台对目标 ±0.02）。
- 预检 4 处：平台 160.698 / 159.191 / 160.837 / 159.306，平面度全 0.000，剩余壁 1.868 / 1.373 / 1.895 / 1.394；旧壳反例 4/4 FAIL。报告 `docs/reports/壳柱锪平_2026-09-15.md`。
- Gate 数据未改：F20 stack/engagement、T02/T03 新 counterbore 特征条目等 build 后补。

### 2026-09-15 H01 上半座支撑分析（主线程，只出方案）
- `docs/reports/H01半座支撑方案_2026-09-15.md`：H01-F05 是桥板底面的两段拱形天花板（A 端只顶点 ±20° 贴 Ø22.18、拱上 2.8 mm；B 端 ±45°、拱上 1.6 mm）；声明朝向支撑贴面 798 点；可撕桥/尖拱都不成立；推荐"正着打 + B 座清座"并在最后一次 build 给 B 座顶 ±30° 让位 0.15。
- 注：Gate L1 对 F05 的禁撑区计数含穿过拱下空腔、不碰面的支撑柱（轴竖直时 10472 区内 vs 139 贴面），判据"体积内采样"对半开座偏严 —— 交 Gate 修复 agent 评估是否改成贴面判。

### 2026-09-15 装配前置：舵机法兰/角孔先开牙（主线程，实物反馈）
- 实物：TB02 摆臂 6 孔对位正确（**分度圆 Ø10.5 实物定案**），但 M2×10 自攻拧不进——露出 2.5 mm 减锥形导入 ~1 mm，切不动 Ø1.7 光孔。
- `assembly.yaml` 新增 `pre_assembly_2026-09-15`（装配前所有 S288 法兰 6 孔 + 背面角孔用同款自攻空拧到底开牙，或 M2 丝锥）；`03_采购/采购清单.md` 加 M2 丝锥（可选）；`07_舵机测试台/README.md` 步骤 4 加开牙。
- 未改任何几何与 fasteners 判据；F02（露出 2.0）等短咬入组在开牙后才成立，这一前提写进了 pre_assembly.why。
- 09-15 补：用户实测该批"M2×10 自攻"总长 9.8、帽底到尖 ≈8.4（按总长标），露出 0.9 mm 为无牙锥尖——拧不进的真因是**螺丝长度口径**，不只是开牙。`fasteners.yaml` 顶部加长度口径注释；采购清单第一行加"下单前量帽底到尖，按总长标的买大一号"；测试台 README 步骤 4 加量螺丝/沉窝救急。几何未动。
- 09-15 实物称重：`components.yaml:servo_s288.mass_g` 19.5 assumed → **20.33 measured**（1 颗，含 measured_2026-09-15 子键：舵机+摆臂+3 螺丝 28.10 / 3 螺丝 0.52 / 摆臂 7.25）。`duckstructure/s288.py:mass_g` 仍 19.5（源码质量参数改动要随下次 build 一起，避免 MJCF hash 单独变）。

### 2026-09-16 S288 力矩上限实测回填（主线程，台架）
- `tolerances.yaml:load.servo_rated_torque_Nm.measured_stall_torque_Nm` null → **0.66**（src `bench_2026-09-16`，吊 960 g 抬臂两次都停在离垂直 46.9°；带 4 条 caveats）。原始数据 `V2_S288版发布产物/04_电机到货测试/data_raw/push/lift_m960_*.json`。
- 同日早些时候曾按手扶堵转的舵机**回报值** 0.996 回填过一版，吊瓶证明回报值饱和后是指令回声（轴上只有 0.66），已改正；caveat 里明写"不许拿 1.0 当上限"。
- `bench.yaml:BN13` 加 `partial_result_2026-09-16`：判据①满足（≥0.54），判据②舵盘相位未测，`checked` 保持 false。
- 不动检查器；L7 `_rated_torque_Nm` 仍只读 `stall_torque_Nm`（0.6），本次回填不改任何层的判定。`continuous_torque_Nm` / `torque_speed_curve` 仍 null（BN03/BN17 未做）。
- BAM 辨识（rep1/rep2 77.1 g + m049 48.9 g 共 48 条摆动 + 2 条吊瓶）参数在 `04_电机到货测试/params/s288/`，非 Gate 数据，见 `测试台实录_2026-09-15.md`。

### 2026-09-16 MJCF 重生成 + 训练包（主线程，不改 Gate 检查器）
- `tools/sim/make_mjcf.py`：① `<joint range>` 改读 `mjcf_ranges.json:target_range_deg`（frozen 目标区间，09-15 收窄后的 62/±62/44.5），不再读 collision_free（65/47.5）——满足 L6 crit_free ⊆ 与 crit_cover ⊇ 两条；② 外购件质量改读 `components.yaml`（servo 20.33 实称，原硬编 19.5）；③ 新增点质量：香橙派 14 g @(66.8,0,251) 头内立柱尖（head.py 推得）、XT30 1.7 g @(-23.1,0,134.9)（mount 扎带孔中点）；④ `<default class="chosen_actuator">` ← S288 BAM m1（forcerange ±0.706 电机侧 / frictionloss 0.066 / damping 0.0108 / armature 5.5e-4 / kp 1.882 kv 0.0443）；⑤ 新增 `sim/duck_s288/robot_walk_s288_train.xml`（只有 sole/sole_R 凸包作 left/right_foot_collision，其余凸包降为 visual；与上游 robot_walk.xml 同构，mjlab 用）。`robot_walk_s288.xml`（全件碰撞）仍是 Gate L6/L7 读的那份，inertial 与训练变体相同。整机 750.3 g（15 body）。日志 `docs/build_logs/make_mjcf_2026-09-16.log`。
- L7 复跑（`docs/build_logs/gate_2026-09-16_L7_after_mjcf.log`）：14 条 `joint:*/forcerange_vs_servo_rating` 仍红（0.7057 > 名义堵转 0.6；之前是 0.96 > 0.6，非回归）。**交 Gate 修复 agent**：该判据应在 `measured_stall_torque_Nm` 非空时改比实测值，且 BAM 的 forcerange 是电机侧（减 frictionloss 0.066 = 输出侧 0.64 ≤ 实测 0.66）；不许为了过判据把 XML 改成输出侧值。
- 训练包 `sim/duck_s288/mjlab_duck_s288/`（README 在内）：不改 upstream；`Mjlab-Velocity-{Flat,Rough}-DuckS288`。本机无 torch/mjlab，只验到 CPU MuJoCo（站立 + bam.mujoco 后端复现台架，PyPI 1.0.2 与上游锁定的 bam 62bd8ce 源码两版都跑过、结果相同；执行器类已对着 62bd8ce 的 mjlab.py 核过接口）。待显卡机自检。

### 2026-09-16 训练包显卡机验证（主线程，AutoDL 4090，不改 Gate）
- 机器上现成上游 `29e887e`（本地锁 `5946fd9`）：本包接口逐一比对一致，`microduck_velocity_env_cfg.py` 逐字节相同，bam 同锁 `62bd8ce`；直接用现成环境。
- 修一处循环导入（`mjlab_duck_s288/__init__.py` 先 `import mjlab`）：mjlab 顶层回调加载 `mjlab.tasks` 入口时 robot.py 半初始化会撞上；4 种导入顺序验证通过，`list-envs` 有 Flat/Rough。
- 冒烟 64 env × 20 迭代 RC=0、无 NaN、两行执行器日志出现。同设置 XL330 对照：基线回合 18.7→34.8、S288 19.6→22.5，`body_ang_vel` 惩罚 8 倍——kp 2.0 + 动作尺度 1.0 + 初始噪声 1.0 让随机动作顶到 0.706 饱和；物理后果，是否降 kp 看 4096 env 正式曲线。
- 正式 4096 env × 4000 迭代 run-name `s288_kp2` 已启动（结果另记）。
- v1 正式跑 `s288_kp2`（4096 env）1140 迭代停：奖励曲线 300 迭代追上基线，但回放统计（`sim/duck_s288/mjlab_duck_s288/scripts/rollout_stats.py`）：两 hip_yaw 顶在 ±25° 限位 99% 时间、扭矩 0.45/0.69 N·m 饱和 43%/97%、速度指令 0.065 实际 0.01（原地抖脚刷 air_time）。实机 = 舵机持续堵转，不可部署。关节轴线/锚点/HOME 与上游逐项相同，非模型错。
- v2 改动（不动上游、不动 Gate）：`tools/sim/make_mjcf.py` 训练变体补 lower_leg/lower_leg_R/trunk/zz_battery 凸包为 `self_collision_only`（44 个，对应上游 leg×2+power_support；`robot_walk_s288.xml` 逐字节未变，日志 `docs/build_logs/make_mjcf_2026-09-16b.log`）；`mdp_s288.py` 新增 `torque_above_soft_limit_l2`（0.45 N·m 软上限，权重 −5）+ `joint_torques_l2` −0.05；`dof_pos_limits` −1→−3。v2 `s288_kp2_v2` 已启动。
- v2 `s288_kp2_v2` 4000 迭代跑完（RC=0，2 h 20 min）：奖励 133.6（基线 115.5），摔倒 0.08/窗口；回放统计所有关节越软限位 ≤0.4%、腿扭矩均值 0.07–0.18 N·m、>0.65 峰值 ≤3.1%（踝，迈步瞬间）、指令 +0.30 m/s 达到 0.222（基线 0.157）。产物 `sim/duck_s288/runs/2026-09-16_s288_kp2_v2/`（ONNX/检查点/回放视频/日志/README 含部署对齐项）。v1/v2/基线对比视频同目录。
- 补：`tools/sim/make_mjcf.py` 新增第三输出 `robot_walk_s288_play.xml`（训练 XML 物理逐字节相同 + `vis_*.stl` 整件网格视觉按原版涂装配色 + 相机眼睛环，`assets/` 多 52 个 vis 文件 26 MB；Gate 不读）；训练包 play 配置自动用它；机位环境变量 `DUCK_S288_PLAY_{AZIMUTH,ELEVATION,DISTANCE}`。S288 v2 vs XL330 基线同机位对比视频在 runs 目录。

### 2026-09-16 Gate 宗旨对齐：目标行程改由"原版能力"推导（主线程，用户拍板"改吧改吧"）
- 用户定：**能力必须与原版一致，转角/CAD/大小可以不同**。检查发现 L6 的目标行程是 09-15 按我们的件扫出来再收窄填进去的（62/62/44.5）——判据迁就件，失去约束力；而"策略包络"判据 09-09 被 RETIRED。两处一起翻过来：
- `frozen.yaml` 新增 `capability_envelope`（source policy_envelope.json、key all.actual_deg、in-scope 7 个 mode（走路/起立/坐站/捡物/踢球左右/翻身；roller 两个为轮滑脚选配不在清单）、margin 3°、**左右镜像求并**（原版起立主要用右腿：right_knee −83.1 vs left_knee 75.1；能力不分左右、镜像件只能对称切））。
- 新脚本 `tools/gate/derive_target_ranges.py`：`target_range_deg.v = clip(镜像并集 ± margin, 上游 range)`，只写 v/date/reason/src、保留 src_note；`--check` 核对。写回 14 条，值变化 9 条：**left_knee [−90,62]→[−47.5,86.2]、left_ankle [−62,62]→[−31.2,90]、neck_pitch [−62,44.5]→[−90,60]**、right 镜像、hip_pitch/head_pitch/head_yaw 按包络收窄（原版任务用不到的角度降 WARN）。旧文件备份 `data/.frozen.yaml.bak_0916_before_capability`。
- `layers/l6_motion.py`：删 RETIRED 的 `_policy/policy_box`；新增 `_capability/scope_declared`（INFO/unknown）、`<joint>/capability_envelope`（每 mode 包络 ± margin ⊆ 无碰撞区间，BLOCK）、`_capability/box_collisions`（能力盒内碰撞 = 0，BLOCK，证据 = 盒内扫过的姿态数）；能力盒与脚本共用 `derive_target_ranges.capability_box()`（判据一份实现）。顺手改掉注释里"S288 比 XL330 大"（错，同外形档；干涉来自为 S288 重画的特征）。
- 反例：新 `n_l6_capability_envelope.py`（A 包络够不到 BLOCK 且点名 mode；B 只有右侧用到、镜像后左侧同样 BLOCK；C mode 不在 scope PASS；D 对照 PASS；E 无声明 unknown）8/8 过；`n_l6l7_retired_not_stale.py` 从五处改四处（policy_box 已删）；其余 L6 反例 8/8 仍过。
- **L6 全量未重跑**（扫掠分析在占机器，重计算串行）：预期按新目标区间，膝/踝/颈原 09-15 收窄掩盖的碰撞回到 BLOCK，这是对的——它们就是要改件的清单。

## 2026-09-16 L6 全量（能力包络判据首跑）+ 扫掠分析复核
- `docs/build_logs/gate_2026-09-16_L6_capability.log`（65 反例全过 230 s；L6 本体 667 s）：`capability_envelope` 红 5 条 = 左右膝（缺口 21.1°）、左右踝（25°）、颈俯仰（25°），其余 9 关节绿；`_capability/box_collisions` 4499 条/157 件对（4274 条是组合姿态，膝踝颈修好后要重扫才知道剩多少）。
- 扫掠报告 `docs/reports/行程恢复_扫掠体积分析_2026-09-16.md` §9：踝"脚×踝舵机减法做不到"被独立复核推翻——干涉是脚顶面 2 mm 条带（+90° 时 340 mm³）+ 小凸起，法兰特征只是贴面（交集 0）；改 `ANK_SWEEP` 重建即可。**仍未改任何 CAD。**
- 2026-09-16 用户口述 TC02 孔规新读数（现在买的 M2）：Ø2.5 无阻碍、Ø2.4 有阻碍 → 记入 tolerances.yaml:xy_offset_mm_per_side.reading_2026-09-16（v 未改，待确认是否换批螺丝 / 螺丝外径）。若确认：过孔 2.4→2.5、头窝/垫管孔 4.4→4.5。
- 2026-09-16 用户口述：TC04_horn_24（6×Ø2.4 @ r 5.25，PLA）09-15 测 BAM 时已扣在 S288 输出法兰上、**6 颗 M2 全部对孔拧入** → 分度圆 Ø10.5（horn_r 5.25）实物落实（此前只有手册出处）。同片 6 孔 M2 顺穿；TC02 长条 Ø2.5 顺穿 / Ø2.4 卡（薄壁 vs 实料）。用户决定：过孔保持 2.4 不改；材料改 PETG（待重打 PETG 试件包）；自攻底孔打完钻 Ø1.6。
- 2026-09-16 材料：用户定整鸭 PLA → **PETG**（鞋底 TPU 不变）。parts.yaml 17 件 material 改 PETG；printability.yaml 加 materials.PETG（三条 assumed/not_measured，PETG 试件包 `S288_coupons_PETG_2026-09-16.zip` 已发用户）。密度 1.27 不变（原打印清单口径即 1.27）。**PLA 的孔规读数与 L1 标定对 PETG 不作数**，L1 会按记录过期判；PETG 读数回来再更新。
- 2026-09-16 L6 判据：新增 `_capability/real_pose_collisions`（BLOCK）—— 读 `tools/gate/out/policy_steps_2026-09-16/summary.json`（`tools/sim/policy_pose_collisions.py` 按原版策略逐步真实姿态在 placed/ 上布尔），核 placed/steps/脚本三重指纹 + in-scope 清单，过期/缺失 → unknown；`box_collisions`、`combo_collision_inside_free_range` 降 WARN（轴对齐盒是上界，README A2 的已知缺陷到此修掉）。反例 `n_l6_real_pose_collisions.py`（5 断言）新增，`n_l6_capability_envelope.py` 改为期望 box WARN。

### 2026-09-17 电子件实测与无焊接约束（主线程，只改数据不改检查器）
- `components.yaml:bus_adapter`：宇树转接板实测 40×30×8.7（按 9），XT30(2+2) 拓展口实测 12 V 直通；整鸭定用该板。口袋 28×40×11 → x 向要扩 ≥2（待顶板上方净高图）。
- `components.yaml:switch_rocker`：用户确认未购买；总开关方案（电池与转接板之间、DC ≥10 A、T02/T03 侧壳、GPIO 关机键 + overlayroot）。
- `harness.yaml:HB01.power_injection.scheme_2026-09-17_no_solder`：用户硬约束不焊接 → 焊接 Y 结/焊盘 hub 作废，改成品线：PH2.0 只走信号、12 V 从 XT30(2+2) 经 XT30 一分二注入三条支线。
- 主控：Radxa ZERO 3W（带排针版）已推荐（孔距 58×23 = H01 现有立柱、原版头就是给它设计的、40 针有 I²S、双 USB-C），**用户尚未拍板**；拍板后改 sbc 条目、HB04/HB05/HB08 的 to 字段、make_mjcf 点质量。
- **2026-09-17 主控换 Radxa ZERO 3W（用户拍板）**：`components.yaml` 条目 id `sbc_orangepi_zero3w` → `sbc_radxa_zero3w`（harness/bench/make_mjcf/inventory 同步），65×30、孔距 58×23（H01 立柱不动），mass 14 g 改记 assumed 占位（官网无重量）；D-COMP-02 结案。头部后续改件：H03 通风槽、IMU/麦克风插头高度 ≤14 的 IDC 头、以后喇叭位。

### 2026-09-17 features.yaml 扫掠区间与 r3 源码同步（只改文本，不重量）
- 2026-09-16 行程恢复 build r3 改了扫掠刀程，`features.yaml` 五条 sweep_cut 的 spec_verbatim / nominal_source.v / sweep_range_deg / sweep_samples / derived_from / provenance 行号按源码现值改，每条加 `synced_2026-09-17` 留旧值：**L04-F12** 膝 `build_upper_leg() -62..92 n63` → `_build_upper_leg_base() -88.5..92 n73`（lib.py:47 KNEE_CUT_HI、legs.py:169）；**L05-F08** 踝 `sv ±62 n125 / clr ±62 n21` → `-92..62 n155 / n26`（lib.py:36-38、legs.py:194/197）；**L07-F10** 同 clr `±62 n21` → `-92..62 n26`；**T02-F01 / T03-F01** 颈摆 `-60..45、盒 x0..55` → `-92.5..62.5、盒 x−15..55`（trunk.py:141/151/160；zone_bbox_mm x 下限 0→−15）。
- 未动：L02-F08（髋俯仰舵机 ±62 n11，legs.py:76 源码未改）；所有 verified_on_export 实测数（09-15 r2 件的读数，method 只加注"r3 刀程待 r4 重量"）；grow_mm / min_gap / pos / box 等数值。lib.py sweep_of 行号 566-581 → 600-615、NECK_CLR/NECK_HEAD_CLR → lib.py:108/112。
- 新增膝跟刀（KNEE_HEEL_* 髋侧五件绕膝轴削 L05/L06/L07，legs.py:206）和大腿被小腿扫（KNEE_L03_*，legs.py:99 削 L03）两把 r3 新刀**尚无 feature 声明**，待 r4 后按 09-15 补齐流程新增。

### 2026-09-17 髋横滚 6704 取消 —— Gate 数据同步（只改声明/状态，不重量、不跑 build/Gate）
- 依据 `docs/reports/髋横滚轴承取消_2026-09-17.md`：`lib.RINGS[left_hip_roll]=False`、`legs.py:10 HIP_ROLL_DISC_D=20`，build r4（`docs/build_logs/build_2026-09-17_r4.log`）L01 3.3→2.4 cm³、L02 6.3→5.6 cm³，placed/ 不再有 `bearing_left/right_hip_roll.stl`。改前每个 yaml 备份 `.<name>.bak_0917_hiproll`。
- `components.yaml`：`bearing_6704zz` qty 4→2（`qty_was_until_2026-09-17`）、housed_by [L01,L02,L03,L04]→[L03,L04]（旧值留 `housed_by_was_until_2026-09-17`）、新增 `retired_hip_roll_2026-09-17`；`applies_to_which_bearing` / `why_this_cell_stays_red` / `mount`、`bearing_6702zz.added_2026-09-12`（"偏航 6704 与髋横滚 6704 相交"论据成历史）、A2 `replaced_by`（新增 `replaced_by_updated_2026-09-17`）/ `actually_dropped` / `size_caveat`、D-COMP-01 `why_it_matters` 逐处加注，历史文字不删。
- `features.yaml`：按文件先例（旧 H03-F04 删除）**删除**三条 feature（主线程 09-17 指示；先前一版曾标 `status: retired`，因 L2 无 status 门控改为删条）：**L01-F10** bearing_bore『ring_boss(O,Rs): 6704 外圈座 Ø27.15 × 4.2（x 12.75..16.95），法兰过孔 Ø14.9，凸台 OD31 壁 2.5，内腔 Ø26』；**L01-F15** clearance_cut『ring_boss() 内 M7 挡肩刀 cyl(lip_d=24, 1.05) → 舵机局部 x 12.0..13.05』；**L02-F09** bearing_bore『bearing_driven_clearance(Rr): 环 Ø27.5/Ø19.85 × 4.45 (x 12.65..17.1)』—— 三条都是髋横滚法兰侧 6704 的座/挡肩/让位，09-17 髋横滚 6704 取消后几何不存在。counts.features 182→179，by_kind bearing_bore 9→7 / clearance_cut 10→9，by_part L01 15→13 / L02 10→9（parts.yaml feature_count 同步）。**L02-F01** spec Ø19.85 毂 + Ø26 盘 → `driven(ring=False, d=20)` Ø20×3 盘直接坐法兰（`spec_verbatim_was_until_2026-09-17`、`synced_2026-09-17`），`nominal_d_mm` 26→20、`hub_d_mm` 置 null（not_applicable）、derived_from/note 按现源码；**geom.pos / axial_span / hub_axial_span 未改**（旧盘位，待 r4 重量）。L02-F05 note、T01-F24 src_note 加注。
- `parts.yaml`：L01/L02 role / mates_with / houses / notes 加注（r4 体积写在 notes；本文件没有 volume 字段，未新增）；feature_count L01 15→13、L02 10→9；`motion_instances` 的 `bearing_left_hip_roll` / `bearing_right_hip_roll` 两条**删除**（留一行注释）。
- `relations.yaml`：R01 / R02 加 `scope_2026-09-17`；**R03 站位换到膝**：`contact.parties` 改 `bearing_left_knee`(component bearing_6704_left_knee) × `upper_leg`(L03)，旧 parties 留 `parties_was_until_2026-09-17`；`probe_bbox_mm` → null + `probe_bbox_unknown_class: not_measured`（旧窗留 `probe_bbox_mm_was_until_2026-09-17`、rationale 改名 `_was_until_2026-09-17`）；`measured_2026-09-09` 改名 `measured_2026-09-09_hip_roll_was_until_2026-09-17`，新加 `measured: null` + `measured_unknown_class: not_measured`；`finding` 改名 `finding_was_until_2026-09-17`；`quantity` 改口 L03。**注意** `contact.axis/approach` 仍是旧髋横滚的 x/−x，膝轴沿世界 ±y，重量时要改（已写在 `parties_note_2026-09-17`）。`still_has_bearing` 去掉 left/right_hip_roll，移入 `removed_2026-09-17_from_still_has_bearing` + reason；载荷路径 L01 `what`、L02 `to.what` 加注（`to.pos_mm` 旧盘位未改）。
- `assembly.yaml`：文件头注释；`subassemblies.trunk` note、step08 `action`（旧文留 `action_was_until_2026-09-17`）；6704 数量 `6704×2`→`6704×1`（step08 parts + 9 处 already_installed）；present / movers / already_installed 列表里所有 `bearing_left_hip_roll` / `bearing_right_hip_roll` 元素**删除**；步 8 动作组 `L_roll_bearing_into_L01` / `R_roll_bearing_into_L01`（required_motion_groups + motions 两处各 2 条）**整条删除**（mover 只有这颗轴承，删名字后 movers 为空）；两处 `len_mm_note`、两处 I5 note 加注。
- `fasteners.yaml` F04_hiproll_flange joins / selector 加注（螺丝规格不变）；`keepouts.yaml` KO21 `owner_parts_note_2026-09-17`；`bench.yaml` BN11 `scope_2026-09-17`（压装件数 4→2）。
- `tolerances.yaml`：`fits.pla.bearing_outer_seat_6704` / `bearing_inner_hub_6704` 加 `print_orientation_note_2026-09-17`，TC01 / TC01b 加 `measures_note_2026-09-17`（数值、试件规格不改）；`frozen.yaml:P_dict.brg.seat_d/hub_d` src_note 加注；`printability.yaml` L01 `down_world` 行内注释加 09-17 注（朝向、数值不改）。
- **仍待主线程**：① features/relations 的 L02-F01 盘位（geom.pos/axial_span、载荷路径 to.pos_mm）与 R03 膝站位的探测窗/axis 都等 r4/r5 重量；② `printability.yaml:606` 禁撑区统计里仍有 `L01-F10` 一格（09-15 切片实测数据，键即 feature id，未动）；③ `l3_static.py` 文字仍以『4 颗 6704』作历史案例（代码数据驱动，不影响判定）。

### 2026-09-17 build r4 / r5：髋横滚 6704 取消 + 真实姿态区域刀 + H03 通风槽（主线程）
- **r4**（`docs/build_logs/build_2026-09-17_r4.log`）：RINGS left_hip_roll False（`duckstructure/lib.py` 注释 + `docs/reports/髋横滚轴承取消_2026-09-17.md`）；L02 Ø20×3 盘直接坐法兰（`legs.py:HIP_ROLL_DISC_D`）；L01 无座凸台；H03 通风槽 5×(2×10)（`head.py:H03_VENT_*`）；H03 孔4 Ø6 套环随 r4 首次 build。真实姿态判据：r3 17 个缺陷 body 对（最坏 262 mm³/195 姿态、含两颗 6704 互压 132.8）→ r4 10 对（最坏 115 mm³/7 姿态）。`tools/sim/cad_geometry_manifest.json` 去掉两条髋横滚轴承。
- **r5**（`build_2026-09-17_r5.log`）：`lib.region_cut()` 把 `policy_pose_regions.py` 算出的缺陷区域（`duckstructure/data/regions_2026-09-17/`，件 export_local 系 → 零位姿世界系 + minkowski 0.4）当刀：L02 ×5、L04 ×2、L07 ×1、N01 ×2、T01 ×1（只切运动件/远端件，不切原版件与舵机；右件区域镜像到左件）。`policy_pose_compare.py` 加 `--ours/--orig`，`policy_pose_regions.py` 加 `--dir`；每次 build 的真实姿态结果各一个目录（`tools/gate/out/policy_steps_2026-09-17{,_r5}`），`l6_motion._REAL_POSES` 指向 r5。
- 主控 Radxa ZERO 3W（用户拍板）：`make_mjcf` 点质量 id 改 `sbc_radxa_zero3w`（14 g 占位）；整机 727.7 g。
- 待办：L1 PETG 重切片（`printability.yaml` 09-15 切片统计仍按 r3 件、含已删 L01-F10 格）；L02-F01 geom.pos / relations R03 膝探针窗口 r5 后重量；Gate 全量跑。
- **2026-09-17 L6 `_capability/real_pose_collisions` 判据改为"与原版对比"**：09-16 版是"任何真实姿态碰撞 = BLOCK"，但原版自己在同一姿态里也有靠接触/仿真压入（H03×N02 59.6 mm³、头 jaw×躯干/脚等 13 对），那些不是件的缺陷；现在读同目录 `compare.json`（`policy_pose_compare.py`，新增 fingerprint：ours/orig poses.jsonl + 脚本 sha256），**我们撞而原版不撞的 body 对 = 0** 才 PASS，缺 compare / 指纹旧 → unknown。反例 `n_l6_real_pose_collisions.py` 由 5 条改 7 条（新增 F 缺 compare、G compare 指纹旧；E 改成"有共有碰撞但 0 缺陷对 → PASS"）。`n_l7_component_claim.py` 对照组期望 measured 改读清单 qty（6704 4→2）。
- 2026-09-17 收尾：Gate 全量 r5 `BLOCKED ✅103 ❌362 ⬜6 🪦1`（`docs/build_logs/gate_2026-09-17_full_r5.log`，红格类别同 09-13 基线，无因改件新增的红）；L6 `train_contact_geom_names / train_sensor_frames` 改读训练变体 `_train.xml`（09-16 起脚底 geom 只在那份里）→ 单跑 L6 两条 PASS（`gate_2026-09-17_L6_r5_b.log`）；18 件按 r5 重切（`slice_2026-09-17_r5.log` 全 rc 0 → `slicing/slice_run.yaml`）。交接见 `docs/交接_2026-09-17.md`。
- **2026-09-17 L6 新判据 `_simmodel/bam_params_sync`**（用户问"摩擦会同步到仿真吗"）：BAM m1 有三处落点——发布目录 `m1.json`、mjlab 包内 `s288_m1.json`（FrictionDRBamActuator 的 DR 中心值）、`robot_walk_s288.xml` + `_train.xml` 的 `chosen_actuator`（MuJoCo 关节层 frictionloss/damping/armature/forcerange）；四个数相对差 ≤0.2% 才绿，缺文件/缺 class = unknown。反例 `n_l6_bam_params_sync.py`（改包内 json → 红、改 MJCF → 红、缺文件 → unknown、真实 → 绿）。现状三处一致（`docs/build_logs/gate_2026-09-17_L6_r5_c.log`：判据 91→92）。配套 `bench.yaml:BN18`（带侧向弯矩的摩擦台架，sim2real 前做）。同日另一 agent 把 `_REAL_POSES` 改指向 `policy_steps_2026-09-17_support_review`（带依赖指纹的重算，进行中）→ 重算完成前 `real_pose_collisions` 为 unknown，**故意不改回**。
- 2026-09-17 宇树客服第二轮回复落数据：`tolerances.yaml:servo_output_bearing.rated_radial_N.cs_reply_2026-09-17`（0.38 N·m 长期 OK 为口头 claim，v 仍 null；支承方式 pending）、`bench.yaml:BN02.regen_note_2026-09-17`（制动回馈过压，查电池保护板）、训练计划编码器量化改输出端 8192 步/圈；问题清单文档加第二轮表。
- **2026-09-17 深夜 轴承恢复（接手 Codex 会话 hr01–hr05，续做 hr06–hr09）**：用户要求把原版 6 处轴承加回（髋横滚×2、髋俯仰×2、头偏航、头横滚 A）。结论与实施见 `docs/design_2026-09-17_bearing_rebuild/记录.md` 10–14 条：Codex 的"可拆座+压盖+耳"方案在髋俯仰（膝弯 ≥72.5° 踝舵机撞耳）和髋横滚（外侧根/耳扫掠切掉 L02 俯仰盘 F06 坐面）都放不下，改为 **L03/L01 整体座（6703×2+2，与膝同做法）**；头部按 Codex 补丁（A 端 6704+N05、偏航 6704+N06/N08、B 端 N04 加键+N07）并入并修 4 处（H01 让位 H03 半座、N03 B 端残料、N06 装入通道、F17 起子通道穿偏航座根）。件：L08/L09/L11/L12 退役，L10/L13 轴套，N05–N08 新增 → 24 种打印件；轴承 13 颗（6704×4：膝 2 + 头 A + 头偏航；6703×4：髋横滚 2 + 髋俯仰 2；6702×2；6700×2；22×16×4×1）。`sync_declarations.py --apply` 已同步 parts/features/components/manifest。已知未清项：L02 俯仰盘 180° 那颗 F06 坐面 61/72（L01 偏航盘区横滚扫掠，r5 起即有）；H03 非水密 2 边（hr09 起 clean_print_topology 加 float32 往返修）。


### 2026-09-24 hr39f 电池顶盖/后门接续（工作版，未发布）

- 补齐B03及B01/T01/T02/T03新特征、装拆顺序、F21默认0可选1、F31/F32卡合、KO22、BN19/20；31件/293特征。XT30组合占位包含平衡头，通道顶从漏计的154纠正为157.3；保持力、软线和部分质量仍未知。
- 完整build后按原外皮裁切B03后卡舌根部凸出，B03单件快速重建；两次exit2，机械审计0失败、静态0、全域扫掠210/旧域0。修后B03 2.414872846cm³，未更新发布STL/源码；最终placed快照hr39f。
- 真切片五件全部rc0，fresh STL/Gcode hash及支撑指标仅合并这五行；限定L1最终46 PASS/18 FAIL，仍未通过。B03正式禁撑区空清单明确未知，额外Gcode卡合区支撑邻近诊断保留实物待验。
- 独立复审修正prism反向深度起点、实例pos去重、B03根部名义盒与代表点作用域。修复本轮scalar YAML替换辅助器并清除printability重复旧键。继承F27_camera_to_H04三处重复键留待头部数据处理。
- 证据入口：docs/design_2026-09-17_bearing_rebuild/hr39f_work/接续_2026-09-24.md；本轮未更改阈值、未豁免弹性接触、未覆盖全量Gate scorecard。
