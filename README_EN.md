# microduck-s288

English · [中文](README.md)

A rebuild of Pollen Robotics' open-source bipedal duck [Microduck](https://github.com/pollen-robotics/microduck),
redesigned around the **Unitree S288** servo.

![Exploded view: apart / together](docs/explode.gif)

<sub>Printed parts, servos and bearings, exploded by sub-assembly and reassembled. Interactive version (drag to rotate): [docs/explode.html](docs/explode.html), download and open in a browser.</sub>

> **Status (2026-10-01): parts are at the print shop; the robot is not assembled yet.** This release publishes the
> **CAD generator source, the Gate check framework and all design data** (see "Repository layout"). The gate is not fully
> green: the remaining reds are mostly items that can only be registered from real parts (driver reach, seating faces,
> measured component masses) and collision pairs the original design also has.
> **Do not print or buy anything based on the current state**; STLs, BOM and assembly order follow once the robot stands (see Roadmap).

---

## Why a rebuild

Microduck's simulation model, training code and policies are open, but the **CAD is not**, and the Dynamixel XL330
servos are hard to source in China. So this project:

- **borrows** the simulation skeleton and policies: all 14 joint axes are kept exactly, and the original walking /
  stand-up / sit-stand policies serve as the capability baseline;
- **redesigns** every structural part for the Unitree S288 (plastic gears and housing, Ø10.5 pitch circle,
  measured 0.66 N·m stall). Head shells, hips, yaw links and torso shells are machined from the original meshes as stock;
  the head is widened by 10 mm per side to fit the compute board, camera and speaker;
- **retrains**: with new structure and new actuators, the policy is retrained on a MuJoCo model generated from our CAD,
  not bent to the original joint limits.

## Component choices (final)

| Part | Choice | Notes |
|---|---|---|
| Servos | Unitree S288 × 15 | incl. the beak; one bus feed into 5 splitters, 6 daisy chains; connectors on the rear face, plugged axially (verified on hardware) |
| Compute | Radxa ZERO 3W | in the head; camera, NPU and USB networking already running on the board |
| IMU | Adafruit 4438 (LSM6DSOX, STEMMA QT) | torso front wall, I²C |
| Camera | Radxa Camera 8M 219 (IMX219) | face plate |
| Ranging | VL53L5CX multi-zone ToF | face plate, obstacle avoidance |
| Battery | 3S 1100 mAh 30C, XT30 | rear torso bay, loaded through a door; unplugged when not in use |
| Power | 12 V direct to the servo bus; UBEC 5 V/3 A for compute | |
| Material | mostly PLA; PETG where it gets warm (head bracket, head shells, amp / adapter trays); TPU soles | |

## Repository layout

```
docs/explode.gif, docs/explode.html   the exploded-view animation above, and its interactive version
docs/gate/                            gate verification notes and an independent audit
duckstructure/                        parametric CAD of the whole duck (Python: trimesh + manifold), one file per body region; data/ holds clearance-region meshes and the head-scaling cache
tools/cad/                            assembly / mechanical audits, hole and cleanliness checks (shared by build and gate)
tools/gate/                           the 8-layer gate: gate.py entry point, layers/, data/ (design data YAML), negatives/ (regression counter-examples), slicing/ (slicer-based checks)
tools/sim/                            MJCF generation, real-pose sampling with the original policies, motion guard
scripts/fetch_upstream.sh             clones the Pollen upstream repos into upstream/ (their files are not redistributed here)
requirements.txt                      pinned dependencies
```

**The eight gate layers**: L0 mesh validity → L1 printability (real slicing) → L2 "did the feature actually get cut" →
L3 zero-pose interference → L4 assembly paths and screwdriver access → L5 screw engagement and seating →
L6 full-range and real-pose collisions → L7 mass and torque. Each criterion is an invariant; every red/green cell carries
evidence and provenance. `negatives/` holds the counter-examples: every criterion has a "deliberately broken part must go red"
regression, and a criterion change must pass them first. Details in [tools/gate/README.md](tools/gate/README.md) (Chinese).

### Running it

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
scripts/fetch_upstream.sh                      # upstream simulation skeleton and policies
.venv/bin/python -m duckstructure.build        # generates cad/duck_s288/ incl. placed/ meshes (tens of minutes, memory hungry)
.venv/bin/python tools/gate/gate.py            # full gate; results in tools/gate/out/ (negatives take ~7 min, layers extra)
```

L1 slicing needs a local PrusaSlicer (`tools/gate/slicing/slice_l1.py --exe <path>`); without it that layer reports unknown rather than pretending green.

## What exists today

- **CAD**: 37 part types / 46 printed parts, fully parametric; the 8th print package is at the print shop (left foot printed first as a test).
- **Gate**: the full chain runs; a few hundred reds remain — mostly registrations that need real parts (driver reach, seating faces,
  measured masses) and collision pairs the original design also has. Every cell carries its provenance in the scorecard.
- **Simulation / training**: MuJoCo model regenerates from our CAD; walking / stand-up / sit-stand policies trained once in simulation;
  the head is heavier than at training time, so a mass-limit test is in progress.
- **Hardware**: servos, compute board, camera, IMU and battery in hand; board flashed, networked, camera and NPU inference running;
  the on-board runtime only reads sensors so far, no torque yet; **nothing assembled**.

## Roadmap (uploads follow these milestones)

1. ☑ README + licenses
2. ☑ CAD source, gate framework and design data (this release; full-green gate and independent review to be completed while the robot is assembled)
3. ☐ Printed parts assembled and standing → STLs, print list, BOM, assembly order
4. ☐ Policy retrained, robot walks → simulation scene, weights, training config
5. ☐ On-board runtime (IMU, bus driver, policy inference)

## License

- **Code** (`duckstructure/*.py`, `tools/`, `scripts/`): [Apache License 2.0](LICENSE)
- **Hardware / 3D models and design data** (`duckstructure/data/`, `tools/gate/data/`, meshes and renders under `docs/`): [CC BY-NC-SA 4.0](LICENSE-HARDWARE.md).
  The original models are CC BY-SA-NC and ours are derivative works: **no commercial use**, attribution to Pollen Robotics required.
- Attributions in [NOTICE.md](NOTICE.md). The Unitree S288 manual is copyrighted and is not redistributed here.

## Contact

Issues. Early-stage project; questions welcome, with the understanding that it cannot be built yet.
