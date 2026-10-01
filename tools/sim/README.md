# 本地策略回放的 CAD 预检

`render_policy.py` 现在在写入 `data.ctrl` 前执行强制预检。预检拒绝时退出 **2**，保留 JSON 中的关节、姿态、路径比例、相交零件和 mm³；不会把目标逐轴夹角后继续运行，也没有跳过预检的命令行选项。

从仓库根目录用 `./.venv/bin/python`：

```sh
# 仅在完整 CAD 构建结束、STL 稳定后记录源身份。这个操作不表示几何通过。
./.venv/bin/python tools/sim/cad_motion_check.py snapshot

# 真实回放入口，所有输出检查在本机执行，不打开串口。
./.venv/bin/python tools/sim/render_policy.py \
  sim/runs/2026-09-06_baseline_xl330/baseline_xl330.onnx \
  sim/guarded_policy.mp4 --seconds 5 --no-video \
  --report docs/repair_2026-09-09/motion/replay_preflight.json

# 也可以先检查已记录的目标序列；遇到第一处问题停止。
./.venv/bin/python tools/sim/cad_motion_check.py trajectory \
  docs/six_checks_2026-09-08/policy/forward.npz \
  --report docs/repair_2026-09-09/motion/forward_preflight.json

# 轻量回归，不创建真实 CAD、不操作硬件。
./.venv/bin/python -m unittest discover -s tools/sim -p test_motion_guard.py -v
```

实际生效的检查：

- 关节顺序和全部 14 个关节范围直接读 `robot_walk.xml`；不使用旧 CAD 窄窗口。目标、当前姿态和上次目标必须全维、有限且符合范围。右髋横滚的 23.093° 目标会被拒绝，MJCF 上限为 22°。
- 用 `placed` 世界零位网格和完整运动学链做联合姿态布尔检查，阈值保持 **0.05 mm³**。该阈值只是数值检查阈值，不是可以接受的物理干涉。
- 每次提交检查“当前姿态→目标”和“上次目标→新目标”两条关节线性过渡；最大单关节采样间距 **0.5°**，短移动也检查中点。每个 MuJoCo 物理步后还检查观测姿态间过渡，发现偏离造成的冲突则停止后续积分。
- 固定在同一刚体内的适配件也在启动时检查。剩余三个原版头部原件之间的 3 个配对沿用主 CAD 明示排除范围。H03只对上头壳保留一个额外白名单：每次加载都从原版下头壳计算原始交集，确认 H03 的交集没有超出原始接合区（数值误差阈值0.001 mm³）。H03与舵机、支架、脸件和下颚等其他配对正常检查；白名单与原版对比数字写入报告。
- `cad_geometry_manifest.json` 绑定完整对象清单及 STL、运动学和相关源码的 SHA256。缺件、多件、源散列不符、运行中源文件改动、布尔异常或无效结果均拒绝。运行时检查文件的 inode、大小、mtime、ctime；重新启动时完整核对散列。缓存不能绕过源变化检查。

`MotionGuard.apply_target()` 是可复用入口，但它只实现离线的预检。回放明确选用 **scene_walk.xml→robot_walk.xml**，与六项补查的基线一致；原入口所用的 `scene.xml→robot_allcollisions.xml` 不再被间接默认选择。物理步长0.005 s、implicitfast、10次接触求解迭代都写入报告。它仍是 **原 XL330 XML 执行器动力学**，没有添加 S288 整机驱动，也没有声称已接入实机。拒绝意味着停止离线回放，不能把这个处理直接移植成运动中的实机急停策略。

采样通过仅证明已取样姿态没有超过阈值的相交：它不证明连续时间无碰撞、任意执行器异步响应安全、制动可行、打印公差、轴承负载、线束或电路安全。模型有固定干涉时，正确行为是启动即拦截；源身份快照不是绕过这个拦截的许可。
