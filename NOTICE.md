# NOTICE —— 来源与致谢

microduck-s288 建立在下列开源工作之上。分发本仓库或其衍生作品时请保留本文件。

## Pollen Robotics — Microduck

- 代码：<https://github.com/pollen-robotics/microduck>、<https://github.com/pollen-robotics/microduck_rl>（Apache License 2.0）
- 3D 模型：CC BY-SA-NC
- 本项目**借用**：MuJoCo 仿真模型中的关节轴线与骨架尺寸（一根不动）、训练好的行走 / 起立 / 坐站策略作为能力基线、
  头壳 / 髋 / 偏航件 / 躯干壳等原版网格作为改造毛坯。
- 本项目**重做**：全部结构件按宇树 S288 舵机重新设计（原版用 Dynamixel XL330）。

## Antoine Pirrone — Open Duck Mini

- <https://github.com/apirrone/Open_Duck_Mini>、<https://github.com/apirrone/Open_Duck_Playground>（Apache License 2.0）
- Microduck 的前身，运行时与训练环境的参考。

## AI-FanGe — Microduck build tutorial

- <https://github.com/AI-FanGe/Microduck-build-tutorial>（MIT License）
- 原版 Microduck 的国内复刻教程，本项目选型初期的对照参考；未使用其文件。

## Unitree（宇树）— S288 舵机

- 舵机参数与尺寸取自宇树官方《S288 使用手册》。**手册受宇树版权保护，本仓库不再分发**，请从宇树官方渠道获取。
- 本项目在原版手册基础上做了实测校正（分度圆、堵转扭矩、插座位置等），校正值在数据文件中逐条注明来源。

## 其他

- 仿真：[MuJoCo](https://mujoco.org/)（Apache 2.0）、[mjlab](https://github.com/mujocolab/mjlab)
- 几何：[trimesh](https://trimesh.org/)、[manifold3d](https://github.com/elalish/manifold)
