# microduck-s288

English · [中文](README.md)

A rebuild of Pollen Robotics' open-source bipedal duck [Microduck](https://github.com/pollen-robotics/microduck),
redesigned around the **Unitree S288** servo.

> **Status (2026-09-22): design phase, no physical robot yet.** CAD for all 24 printed parts exists and passes the
> full geometry gate, but the robot has never been assembled, the policy has not been retrained, and dimensions that
> depend on real parts are still nominal. Only this README and the licenses are published for now; STLs, source and
> BOMs will follow as milestones are reached (see Roadmap). **Do not print or buy anything based on the current state.**

---

## Why a rebuild

Microduck's simulation model, training code and policies are open, but the **CAD is not**, and the Dynamixel XL330
servos are hard to source in China. So this project:

- **borrows** the simulation skeleton and policies: all 14 joint axes are kept exactly, and the original walking /
  stand-up / sit-stand policies serve as the capability baseline;
- **redesigns** every structural part for the Unitree S288 (plastic gears and housing, Ø10.5 pitch circle,
  measured 0.66 N·m stall). Head shells, hips, yaw links and torso shells are machined from the original meshes as stock;
- **retrains**: with new structure and new actuators, the policy is retrained on a MuJoCo model generated from our CAD,
  not bent to the original joint limits.

## Component choices (final)

| Part | Choice | Notes |
|---|---|---|
| Servos | Unitree S288 × 14 (+1 spare) | 3 daisy chains; connectors on the rear face, plugged axially (verified on hardware) |
| Compute | Radxa ZERO 3W | in the head, 40-pin header facing up |
| IMU | Adafruit 4438 (LSM6DSOX, STEMMA QT) | torso front wall, I²C |
| Camera | Radxa Camera 8M 219 (IMX219) | face plate |
| Battery | 3S 1100 mAh 30C, 72 × 25 × 18 mm, XT30 | rear torso bay, loaded through a door |
| Power | 12 V direct to the servo bus; UBEC 5 V/3 A for compute | |
| Material | PETG (structure) / TPU (soles, soft parts) | |

## What exists today

- **CAD**: 24 printed parts, fully parametric in Python (trimesh + manifold). Change one dimension, regenerate the whole duck.
- **Gate**: an 8-layer automated check — mesh validity, printability, "did the feature actually get cut", zero-pose
  interference, assembly paths and screwdriver access, screw engagement and seating, full-range and real-pose
  collisions, mass and torque. Every part change runs the full gate; every red/green cell carries evidence.
  Current round: the last head-shell interference seen in 5,601 real policy poses has just been relieved and is being verified.
- **Simulation**: MuJoCo model regenerates from our CAD; the retraining pipeline is set up but has not been run.
- **Hardware**: servos in hand; bus communication, zeroing and a hanging-weight torque test have been done; compute, camera and IMU ordered; **nothing assembled**.

## Roadmap (uploads follow these milestones)

1. ☑ README + licenses (this)
2. ☐ Gate green, independent review with no blockers → CAD source, gate framework and data files
3. ☐ Printed parts assembled and standing → STLs, print list, BOM, assembly order
4. ☐ Policy retrained, robot walks → simulation scene, weights, training config
5. ☐ On-board runtime (IMU, bus driver, policy inference)

## License

- **Code** (CAD generators, gate framework, data files): [Apache License 2.0](LICENSE)
- **Hardware / 3D models** (printed parts, renders, lists): [CC BY-NC-SA 4.0](LICENSE-HARDWARE.md). The original
  models are CC BY-SA-NC and ours are derivative works: **no commercial use**, attribution to Pollen Robotics required.
- Attributions in [NOTICE.md](NOTICE.md). The Unitree S288 manual is copyrighted and is not redistributed here.

## Contact

Issues. Early-stage project; questions welcome, with the understanding that it cannot be built yet.
