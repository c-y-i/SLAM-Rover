# Python Scripts — Agent Guide

Authoritative guide for agents and contributors working in `py_scripts/`.

The shared viewer workspace lets multiple sensor projects reuse one host-side Python runtime, scene system, and dependency set instead of copying viewer code into each firmware project.

## Purpose

- Single source of truth for reusable Python viewer logic
- Firmware projects keep thin wrappers and launch scripts; real code lives here
- Safe to add sensor support without duplicating rendering or runtime code

## Ownership Boundaries

Shared code belongs in `py_scripts/` when it is reusable across projects:

- `py_scripts/requirements.txt`
- `py_scripts/sensor_viewers/` — viewer implementations per sensor

Project-local code stays in each sensor project:

- PlatformIO firmware
- Sensor wiring details
- Project launch scripts
- Device-specific serial protocol

Note: `rover_stack/py/` now contains **compatibility wrappers** that forward to `py_scripts/rover_tools/`. Shared dependencies still come from `py_scripts/requirements.txt`. The SLAM package lives at `slam/` (repo root), not in `py_scripts/`.

## Layout

```
py_scripts/
├── requirements.txt
├── rover_tools/          teleop + record + replay tools for rover workflows
└── sensor_viewers/
    ├── ld06_viewer/         2D top-down viewer for LD06_lidar
    └── vl53l5cx_viewer/     3D point cloud viewer for VL53L5CX_tof
```

## Environment

```bash
cd py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Testing Expectations

When changing shared viewer code:

- run `python -m compileall` as a syntax/import check
- confirm README run instructions still match reality
- validate both `VL53L5CX_tof` and `LD06_lidar` viewers independently

## Rules

- Prefer behavior-level reuse over copy-paste
- Do not silently migrate a project from local viewer logic to shared logic without updating docs
- Update `py_scripts/README.md` when shared layout or usage changes
- Update the consuming project README if the shared setup or launch flow changes
