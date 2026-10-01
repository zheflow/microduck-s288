#!/usr/bin/env bash
# 拉取 Pollen Robotics 的上游仓库到 upstream/（本仓库不分发它们的文件）。
# duckstructure 的关节轴线取自 microduck_rl 的 robot_walk.xml，头壳放大缓存校验上游网格的 sha；
# tools/sim 的真实姿态采样用 microduck/policies 里的 ONNX 策略。两者都是 Apache-2.0。
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p upstream
fetch() {  # 名字 地址 提交
  if [ ! -d "upstream/$1/.git" ]; then git clone --quiet "$2" "upstream/$1"; fi
  git -C "upstream/$1" fetch --quiet origin "$3" 2>/dev/null || git -C "upstream/$1" fetch --quiet origin
  git -C "upstream/$1" checkout --quiet "$3"
  echo "upstream/$1 @ $3"
}
fetch microduck_rl https://github.com/pollen-robotics/microduck_rl 5946fd9
fetch microduck    https://github.com/pollen-robotics/microduck    9f7eaad
