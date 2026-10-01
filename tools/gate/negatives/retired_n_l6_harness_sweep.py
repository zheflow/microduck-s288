# ── 已退役（2026-09-26，hr46 E 段）────────────────────────────────────────────────────────────
# 用户原话（09-26 晚）：『注释掉吧，我已经开始打印了，后面我会再找你，有问题那个时候再修吧。』
# L6 的 E′ 线束实体模型扫掠（_harness_sweep）调用已注释掉（见 tools/gate/layers/l6_motion.py 文件头「E′ 停用」），
# 本反例验的就是那段 → 整文件注释并改名 retired_n_l6_harness_sweep.py（反例运行器只认 negatives/n_*.py，改名即不再发现；
# 不留恒绿反例）。恢复：改回 n_l6_harness_sweep.py，每行去掉行首一层 "# "，删掉本段文件头。
# 停用前原文：docs/design_2026-09-17_bearing_rebuild/hr43_work/hr46/before/E_2026-09-26_user_disable/tools/gate/negatives/n_l6_harness_sweep.py

# #!/usr/bin/env python3
# """反例（hr44reg 2026-09-25）—— L6 E′ harness_sweep：线束模型区内线∩件 / 弯半径不够必须红在该线路格；调不通必须 NOT_RUN(BLOCK)、层不崩。
#
# 背景：hr44 起过颈 / 躯干 / 腿的线有实体模型（duckstructure.wiring_body），L6 按 sweep_summary(coarse=False) 逐线路判
# （协调员 23:15 契约）：PASS ⇔ hit_in_target == 0 且 tension_in_target == 0 且 min_bend_R ≥ min_bend_limit，否则 FAIL(BLOCK)；
# 函数缺失 / 抛异常 / 替身场景 → NOT_RUN(BLOCK)（协调员 00:50：绝不 INFO，免得"没跑"读成放行 = 空绿）。
#
# 造法（_l6_scene 口径：替身 placed/ 全清单 1 mm 立方 + 合成 evaluate 恒 0 + 只扫 head_yaw 一条关节，只为让 l6_motion.run 跑得快）：
# 猴子补丁 duckstructure.wiring_body.sweep_summary（返回合成结果，登记线路 = harness.yaml:bundles[].route_model.routes）与
# _placed_dir（指到替身目录 —— 层的"同一场景"守卫：wiring_body 扫的 placed/ 必须就是本层 placed/）。停在哪一步：不停，run 跑到底；
# 断言全部落在 l6_motion.run 发出的 finding 上。
#   A 对照  全部线路 0 / 0 / R ≥ 限 → 每条 L6/<线路>:harness_sweep PASS(BLOCK) 且 evidence_n = n_poses；HB09 两支都判了 → L6/HB09_cross_neck 不再发『没有折线』unknown
#   B 坏样本 HB09_R hit_in_target=3 → L6/HB09_R FAIL(BLOCK) measured={hit_in_target: 3, …}；bus_L_2to3 min_bend_R 2.38 < 3.0 → FAIL(BLOCK)；
#           其余线路仍 PASS；层结果交给 gate._layer_guard → 不出 L6/_layer_error
#   C 抛异常 → 登记线路全部 NOT_RUN(BLOCK)，detail 带异常类型 RuntimeError；层不崩
#   D 没有 sweep_summary 属性 → NOT_RUN(BLOCK) AttributeError
#   E 替身场景但 _placed_dir 没补丁 → NOT_RUN(BLOCK) PlacedMismatch，且 sweep_summary 一次都没被调用（不拿真 placed/ 的结果冒充替身场景）
# 旧 l6_motion（只有『没有折线』unknown、不调 wiring_body）：B 里 L6/HB09_R 根本没有格 → 本反例红。
# hr46（2026-09-26，主设计 + Gate 守护拍板，只加不减）：两桶证据（own = 撞本线路自己登记的 hook 钩实体 / other = 其余）。合成行补 6 个计数新键
# （默认 own 0 / other = 合计 / both 0）+ own_fixing_files（登记了 hook 的线路 = 按 anchors 拼的期望表，替身钩文件放在替身 placed/ 旁 hooks/）
# + own_fixing_missing [] + identity_ok；B/F 期望 measured 补 6 键。新增：
#   I  sweep_summary 报 missing 且替身钩文件真删掉 → L6/HB09_L NOT_RUN(BLOCK) HookMissing，detail 点名文件；I2 summary 说齐了但文件不在 → 本层 is_file 抓到，同 I
#   J  缺两桶新键 hit_real_poses_own → L6/HB09_R NOT_RUN(BLOCK) MissingKey
#   K  summary 报 own_fixing_files=[]，但 anchors 给 HB09_R 登记了 hook → NOT_RUN(BLOCK) HookMismatch（本层不只信 summary）
#   L  恒等式不成立：HB09_R 计数对不上但 identity_ok 硬写 True、HB09_L 计数对上但 identity_ok=False → 两格都 NOT_RUN(BLOCK)，detail「恒等式不成立」
#   另：A 里本层自己拼的钩文件与 wire_fixings_v1.json 都进 res.inputs；HB09 格 measured = 原 5 键 + 6 键。
# """
# from __future__ import annotations
# import json
# import sys
# from contextlib import contextmanager
# from pathlib import Path
#
# _HERE = Path(__file__).resolve()
# sys.path.insert(0, str(_HERE.parent))
# import _l6_scene                                                                 # noqa: E402,F401  先设 sys.path
# import core                                                                      # noqa: E402
# from _harness import findings, assert_red, assert_green, result, summarize        # noqa: E402
# from _l6_scene import dummy_placed, scene_class, run_l6, restrict_joints          # noqa: E402
#
# NAME = ("线束模型扫掠（HB09_R 肢体区内线∩件 3 / bus_L_2to3 零位弯 R 2.38 < 3.0 / bus_L_3to4 只真实姿态被脚压 / 缺新键 / 端到端小腿线）："
#         "L6/<线路>:harness_sweep 必须 FAIL(BLOCK)；缺新键与调不通 NOT_RUN(BLOCK) 不崩")
# EXPECT = ("B：L6/HB09_R、L6/bus_L_2to3:harness_sweep FAIL(BLOCK)（measured 三个数），其余线路 PASS；A 全 PASS 且 L6/HB09_cross_neck 无 unknown；"
#           "C/D/E NOT_RUN(BLOCK) 带 RuntimeError / AttributeError / PlacedMismatch，E 不调用 sweep_summary；无 L6/_layer_error"
#           "；hr46：I/I2 HookMissing、J MissingKey、K HookMismatch、L 恒等式不成立 → 该线路格 NOT_RUN(BLOCK)")
# BAD_HIT, BAD_BEND = "HB09_R", "bus_L_2to3"
# BAD_REAL, BAD_KEY = "bus_L_3to4", "HB09_L"                                          # hr44 第二轮 F / G
# OWN_R, OWN_L = "HB09_R", "HB09_L"                                                   # hr46 I / J / K / L（登记了 hook 锚点的两条线）
# OWN6 = ("hit_in_target_limb_own", "hit_in_target_limb_other", "hit_in_target_limb_both",
#         "hit_real_poses_own", "hit_real_poses_other", "hit_real_poses_both")        # hr46：两桶 6 键
#
#
# # was_until_2026_09_26_hr46: def _row(hit=0, ten=0, R=3.3, lim=3.0, n=48, hit_real=0, ten_real=0, legacy_hit=None, drop=()):
# def _row(hit=0, ten=0, R=3.3, lim=3.0, n=48, hit_real=0, ten_real=0, legacy_hit=None, drop=(),
#          own=None, own_real=None, files=(), missing=(), identity=None):
#     """hr44 第二轮：带新键（肢体全关节 / 真实姿态）；legacy_hit = 旧口径 hit_in_target（只扫跨越关节，留痕，不参与判）；drop = 故意缺的键"""
#     d = {"n_poses": n, "hit_in_target_limb": hit, "tension_in_target_limb": ten, "hit_real_poses": hit_real, "tension_real_poses": ten_real,
#          "n_poses_limb": n, "n_real_poses": 5601, "limb": "neg", "limb_joints": ["head_yaw"],
#          "hit_in_target": hit if legacy_hit is None else legacy_hit, "tension_in_target": ten,
#          "min_bend_R": R, "min_bend_limit": lim, "od_mm": 5.26, "length_mm": 100.0, "status": "反例合成", "hit_all": hit, "tension_all": ten,
#          "worst_bend_R_swept": R, "crosses": ["head_yaw"]}
#     # hr46：两桶证据（own, other, both；默认 own 0 / other = 合计 / both 0）+ 钩文件表 + identity_ok（默认按计数自算；identity 给了就硬写）
#     lo, lt, lb = own if own is not None else (0, hit, 0)
#     ro, rt, rb = own_real if own_real is not None else (0, hit_real, 0)
#     d.update(hit_in_target_limb_own=lo, hit_in_target_limb_other=lt, hit_in_target_limb_both=lb,
#              hit_real_poses_own=ro, hit_real_poses_other=rt, hit_real_poses_both=rb,
#              own_fixing_files=list(files), own_fixing_missing=list(missing),
#              identity_ok=bool(lo + lt - lb == hit and ro + rt - rb == hit_real) if identity is None else identity)
#     for k in drop: d.pop(k, None)
#     return d
#
#
# def _dummy_hooks(placed):
#     """hr46：替身场景的 own 钩文件 —— L6 按 wire_fixings_v1.json anchors（type hook）的 part + id 在 PLACED 旁 hooks/ 拼名（不 glob），
#     这里照同一规则放 1 mm 立方替身（先清空该目录的旧 STL）。返回 {线路: [相对仓库根的路径（排序）]}；期望表本反例自己算，不借 l6_motion 的函数。"""
#     import trimesh
#     hd = Path(placed).parent / "hooks"
#     hd.mkdir(parents=True, exist_ok=True)
#     for p in hd.glob("*.stl"):
#         p.unlink()
#     js = json.loads((core.ROOT / "duckstructure/data/wire_fixings_v1.json").read_text(encoding="utf-8"))
#     exp = {}
#     for a in js.get("anchors") or []:
#         if isinstance(a, dict) and a.get("type") == "hook":
#             f = hd / f"{a['part']}_{a['id']}.stl"
#             trimesh.creation.box(extents=(1.0, 1.0, 1.0)).export(str(f), file_type="stl")
#             exp.setdefault(str(a["route"]), []).append(str(f.relative_to(core.ROOT)))
#     return {k: sorted(v) for k, v in exp.items()}
#
#
# # H 端到端用的冻结折线（hr44 build #5 的 bus_L_3to4 挂小腿 leg 的一段：离踝插头 11–41 mm，复审 B-3 真实姿态被脚 / 踝后臂压 116 个 >1 mm）
# NEG_FOOT_PTS = [[-47.03, 42.11, 61.91], [-47.03, 39.31, 59.11], [-47.03, 36.5, 56.3], [-47.03, 33.7, 53.5], [-44.87, 31.34, 51.3],
#                 [-41.66, 29.5, 51.3], [-37.7, 29.5, 51.3], [-34.28, 30.25, 50.55], [-30.5, 30.5, 50.3], [-26.53, 30.5, 50.3]]
#
#
# def _e2e_foot():
#     """H：临时线路数据（只这一条，挂 leg、crosses=[]）→ 旧 sweep_check（只扫跨越关节）与新 sweep_limb_real 的命中数（真 placed/）"""
#     import os, json, tempfile
#     import duckstructure.wiring_body as WB
#     rt = {"id": "neg_foot_press", "kind": "single", "members": ["servo_flat"], "stations": [["leg", q, "path"] for q in NEG_FOOT_PTS],
#           "flex": {}, "crosses": []}
#     fd, tmp = tempfile.mkstemp(suffix=".json", prefix="neg_foot_"); os.close(fd)
#     json.dump({"routes": [rt], "junctions": {}}, open(tmp, "w"))
#     saved = WB.DATA
#     try:
#         WB.DATA = tmp
#         old = WB.sweep_check(only=["neg_foot_press"])["neg_foot_press"]
#         new = WB.sweep_limb_real(only=["neg_foot_press"])["neg_foot_press"]
#     finally:
#         WB.DATA = saved; os.remove(tmp)
#     return old, new
#
#
# @contextmanager
# def _patched(wb, summary=None, placed_dir=None, drop=False):
#     keys = ("sweep_summary", "_placed_dir")
#     saved = {k: getattr(wb, k) for k in keys if hasattr(wb, k)}
#     try:
#         if drop:
#             delattr(wb, "sweep_summary")
#         elif summary is not None:
#             wb.sweep_summary = summary
#         if placed_dir is not None:
#             wb._placed_dir = lambda: str(placed_dir)
#         yield
#     finally:
#         for k in keys:
#             if hasattr(wb, k) and k not in saved:
#                 delattr(wb, k)
#         for k, v in saved.items():
#             setattr(wb, k, v)
#
#
# def _hs(r, subject=None):
#     return [f for f in (r.findings if r else []) if f.check == "harness_sweep" and (subject is None or f.subject == subject)]
#
#
# def run() -> dict:
#     import duckstructure.wiring_body as WB
#     import gate
#     base = core.load_data()
#     routes = [str(x) for b in base["harness"]["bundles"] for x in ((b.get("route_model") or {}).get("routes") or [])]
#     data = restrict_joints(base, {"head_yaw"})
#     placed = dummy_placed("harness_sweep", data)
#     scene = scene_class(hit_fn=lambda *a: 0.0)
#     good = {r: _row() for r in routes}
#     bad = dict(good)
#     bad[BAD_HIT] = _row(hit=3)
#     bad[BAD_BEND] = _row(R=2.38, lim=3.0)
#     realonly = dict(good); realonly[BAD_REAL] = _row(hit=0, hit_real=2, legacy_hit=0)          # F：只有真实姿态压到（旧口径 0）
#     nokey = dict(good); nokey[BAD_KEY] = _row(drop=("hit_real_poses",), legacy_hit=0)          # G：缺新键，旧键在
#     # hr46：替身钩文件 + 合成行按 anchors 补 own 钩文件表（登记了 hook 的线路 = 期望表，其余 []）
#     exp_hooks = _dummy_hooks(placed)
#     for rows in (good, bad, realonly, nokey):
#         for r_ in list(rows):
#             rows[r_] = dict(rows[r_], own_fixing_files=list(exp_hooks.get(r_, [])))
#     gone = (exp_hooks.get(OWN_L) or [None])[0]                                               # I / I2 真删掉的替身钩文件
#     miss = dict(good); miss[OWN_L] = dict(good[OWN_L], own_fixing_files=[p for p in exp_hooks.get(OWN_L, []) if p != gone],
#                                           own_fixing_missing=[gone] if gone else [])        # I：summary 报 missing
#     liar = dict(good)                                                                        # I2：summary 说齐了，文件却不在
#     nokey2 = dict(good); nokey2[OWN_R] = {k: x for k, x in good[OWN_R].items() if k != "hit_real_poses_own"}   # J
#     nofiles = dict(good); nofiles[OWN_R] = dict(good[OWN_R], own_fixing_files=[], own_fixing_missing=[])     # K
#     ident = dict(good)                                                                       # L：两种恒等式不成立
#     ident[OWN_R] = _row(hit=5, own=(2, 2, 0), identity=True, files=exp_hooks.get(OWN_R, ()))    # 2+2−0=4 ≠ 5，identity_ok 硬写 True
#     ident[OWN_L] = _row(hit=3, own=(0, 3, 0), identity=False, files=exp_hooks.get(OWN_L, ()))   # 计数对上，identity_ok=False
#     calls = []
#
#     def spy(coarse=False):
#         calls.append(coarse)
#         return good
#
#     def boom(coarse=False):
#         raise RuntimeError("反例：sweep_summary 炸了")
#
#     out, err = {}, {}
#     for tag, kw in (("A", dict(summary=lambda coarse=False: good, placed_dir=placed)),
#                     ("B", dict(summary=lambda coarse=False: bad, placed_dir=placed)),
#                     ("C", dict(summary=boom, placed_dir=placed)),
#                     ("D", dict(drop=True, placed_dir=placed)),
#                     ("E", dict(summary=spy)),
#                     ("F", dict(summary=lambda coarse=False: realonly, placed_dir=placed)),
#                     ("G", dict(summary=lambda coarse=False: nokey, placed_dir=placed))):
#         try:
#             with _patched(WB, **kw):
#                 out[tag] = run_l6(data, placed, scene)
#         except Exception as e:                                              # noqa: BLE001  层崩 = 反例失败
#             out[tag], err[tag] = None, f"{type(e).__name__}: {e}"
#     for tag, rows, rm in (("I", miss, gone), ("I2", liar, gone), ("J", nokey2, None), ("K", nofiles, None), ("L", ident, None)):   # hr46
#         bak = None
#         try:
#             if rm:
#                 bak = (core.ROOT / rm).read_bytes(); (core.ROOT / rm).unlink()
#             with _patched(WB, summary=lambda coarse=False, _r=rows: _r, placed_dir=placed):
#                 out[tag] = run_l6(data, placed, scene)
#         except Exception as e:                                              # noqa: BLE001  层崩 = 反例失败
#             out[tag], err[tag] = None, f"{type(e).__name__}: {e}"
#         finally:
#             if bak is not None:
#                 (core.ROOT / rm).write_bytes(bak)
#     A, B, C, D, E, F, G = (out[k] for k in "ABCDEFG")
#     I, I2, J, K, L = (out.get(k) for k in ("I", "I2", "J", "K", "L"))
#     try:
#         e2e_old, e2e_new = _e2e_foot()
#     except Exception as e:                                                  # noqa: BLE001
#         e2e_old, e2e_new, err["H"] = None, None, f"{type(e).__name__}: {e}"
#     a = []
#     fa = [f for f in _hs(A) if f.subject in routes]                        # 线路格（另有各束的旧『没有折线』格，也叫 harness_sweep）
#     a.append({"claim": f"登记线路 {len(routes)} 条（harness.yaml route_model）≥ 2 且含 {BAD_HIT} / {BAD_BEND}",
#               "ok": len(routes) >= 2 and BAD_HIT in routes and BAD_BEND in routes})
#     a.append({"claim": "A 对照：每条登记线路恰 1 条 harness_sweep，全部 PASS(BLOCK) 且 evidence_n = n_poses 48",
#               "ok": bool(A) and sorted(f.subject for f in fa) == sorted(routes)
#                     and all(f.state == "PASS" and f.severity == "BLOCK" and f.evidence_n == 48 for f in fa)})
#     okG, whyG = assert_green(_hs(A, BAD_HIT))
#     a.append({"claim": f"A 对照 L6/{BAD_HIT}:harness_sweep 绿（PASS 且 evidence_n > 0）", "ok": okG})
#     a.append({"claim": "A：HB09 两支都判了 → L6/HB09_cross_neck 不再发『没有折线』unknown；HB01（部分覆盖）照发且 detail 带 route_model 说明",
#               "ok": bool(A) and not _hs(A, "HB09_cross_neck")
#                     and any(f.state == "FAIL" and "route_model 已登记" in f.detail for f in _hs(A, "HB01_servo_bus"))})
#     fb_hit, fb_bend = _hs(B, BAD_HIT), _hs(B, BAD_BEND)
#     M0 = {"hit_in_target_limb": 0, "hit_real_poses": 0, "tension_in_target_limb": 0, "tension_real_poses": 0}
#     M0.update({k: 0 for k in OWN6})                                          # hr46：measured = 原 5 键 + 两桶 6 键
#     a.append({"claim": f"B：L6/{BAD_HIT} FAIL(BLOCK)，measured = {{hit_in_target_limb: 3, hit_real_poses: 0, tension…: 0, min_bend_R: 3.3}}，evidence_n 48",
#               "ok": len(fb_hit) == 1 and fb_hit[0].state == "FAIL" and fb_hit[0].severity == "BLOCK"
#                     # was_until_2026_09_26_hr46: and fb_hit[0].measured == dict(M0, hit_in_target_limb=3, min_bend_R=3.3) and fb_hit[0].evidence_n == 48})
#                     and fb_hit[0].measured == dict(M0, hit_in_target_limb=3, hit_in_target_limb_other=3, min_bend_R=3.3) and fb_hit[0].evidence_n == 48})
#     a.append({"claim": f"B：L6/{BAD_BEND} FAIL(BLOCK)（零位弯 R 2.38 < 限 3.0），detail 点名",
#               "ok": len(fb_bend) == 1 and fb_bend[0].state == "FAIL" and fb_bend[0].severity == "BLOCK"
#                     and fb_bend[0].measured == dict(M0, min_bend_R=2.38) and "2.38 < 限 3.0" in fb_bend[0].detail})
#     ff, fg = _hs(F, BAD_REAL), _hs(G, BAD_KEY)
#     a.append({"claim": f"F：L6/{BAD_REAL} 只有真实姿态命中（hit_real_poses 2，肢体区内 0，旧口径 hit_in_target 0）→ FAIL(BLOCK)，detail 点名真实姿态",
#               # was_until_2026_09_26_hr46: "ok": len(ff) == 1 and ff[0].state == "FAIL" and ff[0].severity == "BLOCK" and ff[0].measured == dict(M0, hit_real_poses=2, min_bend_R=3.3)
#               "ok": len(ff) == 1 and ff[0].state == "FAIL" and ff[0].severity == "BLOCK" and ff[0].measured == dict(M0, hit_real_poses=2, hit_real_poses_other=2, min_bend_R=3.3)
#                     and "真实姿态 2" in ff[0].detail})
#     a.append({"claim": f"G：L6/{BAD_KEY} 缺新键 hit_real_poses（旧 hit_in_target=0 在）→ NOT_RUN(BLOCK) MissingKey，不回退旧键",
#               "ok": len(fg) == 1 and fg[0].state == "NOT_RUN" and fg[0].severity == "BLOCK" and fg[0].measured is None and "MissingKey" in fg[0].detail
#                     and "hit_real_poses" in fg[0].detail})
#     a.append({"claim": "H：端到端真几何 —— 只挂小腿、不跨关节的线：旧口径（只扫跨越关节）命中 0；新口径真实姿态命中 > 0（被脚 / 踝后臂压）",
#               "ok": e2e_old is not None and e2e_new is not None and e2e_old["n_hit"] == 0 and e2e_new["hit_real_poses"] > 0,
#               "got": None if e2e_new is None else {"old_n_hit": e2e_old["n_hit"], "old_n_poses": e2e_old["n_poses"],
#                                                    "new_hit_real": e2e_new["hit_real_poses"], "new_hit_limb": e2e_new["hit_in_target_limb"],
#                                                    "n_real": e2e_new["n_real_poses"], "limb": e2e_new["limb"]}})
#     a.append({"claim": "B：其余登记线路仍 PASS（红只落在两条坏线路上）",
#               "ok": bool(B) and all(f.state == "PASS" for f in _hs(B) if f.subject in routes and f.subject not in (BAD_HIT, BAD_BEND))})
#     # ── hr46 断言：own 钩文件 / 两桶 6 键 / 恒等式 ──
#     all_hooks = sorted(p for ps in exp_hooks.values() for p in ps)
#     a.append({"claim": f"hr46 A：anchors 给 {OWN_R} / {OWN_L} 登记了 hook（替身钩文件 {len(all_hooks)} 个）；本层自己拼的钩文件全进 res.inputs，wire_fixings_v1.json 也在",
#               "ok": bool(A) and bool(exp_hooks.get(OWN_R)) and bool(exp_hooks.get(OWN_L)) and all(p in A.inputs for p in all_hooks)
#                     and "duckstructure/data/wire_fixings_v1.json" in A.inputs})
#     fa_r = _hs(A, OWN_R)
#     a.append({"claim": f"hr46 A：L6/{OWN_R} measured = 原 5 键 + 两桶 6 键（own 0 / other 0 / both 0），detail 带两桶句",
#               "ok": len(fa_r) == 1 and fa_r[0].measured == dict(M0, min_bend_R=3.3) and "两桶" in fa_r[0].detail})
#     fi, fi2 = _hs(I, OWN_L), _hs(I2, OWN_L)
#     gname = Path(gone).name if gone else "?"
#     for tag, fx, rr in (("I", fi, I), ("I2", fi2, I2)):
#         a.append({"claim": f"hr46 {tag}：登记了 hook 锚点但替身钩文件 {gname} 不在（{'summary 报 missing' if tag == 'I' else 'summary 说齐了，本层 is_file 抓到'}）"
#                            f" → L6/{OWN_L} NOT_RUN(BLOCK) HookMissing，detail 点名文件；其余线路仍 PASS",
#                   "ok": len(fx) == 1 and fx[0].state == "NOT_RUN" and fx[0].severity == "BLOCK" and fx[0].measured is None
#                         and "HookMissing" in fx[0].detail and "登记了 hook 锚点但 hooks/ 里没有" in fx[0].detail and gname in fx[0].detail
#                         and all(f.state == "PASS" for f in _hs(rr) if f.subject in routes and f.subject != OWN_L)})
#     fj, fk = _hs(J, OWN_R), _hs(K, OWN_R)
#     a.append({"claim": f"hr46 J：L6/{OWN_R} 缺两桶新键 hit_real_poses_own → NOT_RUN(BLOCK) MissingKey，detail 点名该键",
#               "ok": len(fj) == 1 and fj[0].state == "NOT_RUN" and fj[0].severity == "BLOCK" and fj[0].measured is None
#                     and "MissingKey" in fj[0].detail and "hit_real_poses_own" in fj[0].detail})
#     a.append({"claim": f"hr46 K：summary 报 {OWN_R} own_fixing_files=[]，但 anchors 登记了 hook → NOT_RUN(BLOCK) HookMismatch（不只信 summary）；其余线路仍 PASS",
#               "ok": len(fk) == 1 and fk[0].state == "NOT_RUN" and fk[0].severity == "BLOCK" and fk[0].measured is None and "HookMismatch" in fk[0].detail
#                     and all(f.state == "PASS" for f in _hs(K) if f.subject in routes and f.subject != OWN_R)})
#     fl_r, fl_l = _hs(L, OWN_R), _hs(L, OWN_L)
#     a.append({"claim": f"hr46 L：恒等式不成立（{OWN_R} 2+2−0≠5 而 identity_ok 硬写 True；{OWN_L} 计数对上而 identity_ok=False）→ 两格都 NOT_RUN(BLOCK)，"
#                        "detail「恒等式不成立」（不许 PASS、不许照旧 FAIL 混过去）",
#               "ok": len(fl_r) == 1 and len(fl_l) == 1
#                     and all(f.state == "NOT_RUN" and f.severity == "BLOCK" and f.measured is None and "恒等式不成立" in f.detail for f in fl_r + fl_l)})
#     guard = None
#     if B is not None:
#         sc = {"cells": {}, "tally": {"PASS": 0, "FAIL": 0, "STALE": 0, "NOT_RUN": 0, "WAIVED": 0, "RETIRED": 0},
#               "blocking_cells": [], "verdict": "CLEAR"}
#         outs = [dict(n=6, mod="l6_motion", name=B.name, result=B, error=None, seconds=0.0)]
#         guard = (gate._layer_guard(outs, {}, [B], False, sc), sorted(sc["cells"]))
#     a.append({"claim": "B 结果交给 gate._layer_guard（全量跑口径）→ 不出 L6/_layer_error", "ok": guard == (([], []), [])})
#     for tag, r, exc in (("C", C, "RuntimeError"), ("D", D, "AttributeError"), ("E", E, "PlacedMismatch")):
#         fs = [f for f in _hs(r) if f.subject in routes]
#         a.append({"claim": f"{tag}：层不崩；登记线路全部 NOT_RUN(BLOCK)（不是 INFO），detail 带 {exc}",
#                   "ok": r is not None and sorted(f.subject for f in fs) == sorted(routes)
#                         and all(f.state == "NOT_RUN" and f.severity == "BLOCK" and f.measured is None and exc in f.detail for f in fs)
#                         and (r.evidence.get("harness_sweep") or {}).get("exception_type") == exc})
#     a.append({"claim": "E：替身场景下真 / 假 sweep_summary 都没被调用（守卫先于调用）", "ok": calls == []})
#     a.append({"claim": "C：没判成 → HB09 不算覆盖，L6/HB09_cross_neck 的『没有折线』unknown 照发（不许因为没跑就把束格撤掉）",
#               "ok": C is not None and any(f.state == "FAIL" and f.severity == "BLOCK" for f in _hs(C, "HB09_cross_neck"))})
#     okR, whyR, red = assert_red(fb_hit + fb_bend + ff, severity="BLOCK")
#     passed = okR and all(x["ok"] for x in a) and not err
#     got = (f"A {len(fa)} 条 {sorted({f.state for f in fa})}｜B {summarize(fb_hit + fb_bend)}｜"
#            f"C/D/E 线路格 {[(k, sorted({(f.state, f.severity) for f in _hs(out[k]) if f.subject in routes})) for k in 'CDE']}｜spy 调用 {calls}｜guard={guard}"
#            f"｜F {summarize(ff)}｜G {[(f.state, f.severity) for f in fg]}｜H 旧 {None if e2e_old is None else e2e_old['n_hit']} / 新真实 {None if e2e_new is None else e2e_new['hit_real_poses']}"
#            + f"｜hr46 I {[(f.state, f.detail[:12]) for f in fi]} I2 {[(f.state, f.detail[:12]) for f in fi2]} J {[(f.state, f.detail[:12]) for f in fj]}"
#            + f" K {[(f.state, f.detail[:13]) for f in fk]} L {[(f.state, f.detail[:15]) for f in fl_r + fl_l]}"
#            + (f"｜崩了：{err}" if err else ""))
#     res = result(NAME, EXPECT, passed, got, red, expect_severity="BLOCK",
#                  detail=(whyR or "") + "｜失败断言：" + "；".join(x["claim"] for x in a if not x["ok"]))
#     res["assertions"] = a
#     return res
#
#
# if __name__ == "__main__":
#     r = run()
#     print(json.dumps(r, ensure_ascii=False, indent=1, default=str))
#     sys.exit(0 if r["passed"] else 1)
