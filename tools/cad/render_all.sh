#!/bin/zsh
# 渲染整鸭装配与各连杆局部视图到 cad/duck_s288/render/
cd "$(dirname "$0")/../.."
BL=/Applications/Blender.app/Contents/MacOS/Blender
R=cad/duck_s288/render; mkdir -p $R
$BL -b --python tools/cad/render_scene.py -- cad/duck_s288/placed "$R/duck" "-" iso,iso2,front,side,back 2>&1 | grep -E "^rendered|Error"
for l in $(ls cad/duck_s288/local); do $BL -b --python tools/cad/render_scene.py -- cad/duck_s288/local/$l "$R/part_$l" "-" iso,iso2 2>&1 | grep -E "Error"; done
echo done
