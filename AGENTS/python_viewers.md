# Python Viewers — Agent Guide

Authoritative guide for agents and contributors working in `python_viewers/`.

The shared viewer workspace exists so multiple sensor projects can reuse one host-side Python runtime, scene system, and dependency set instead of copying viewer code into each firmware project.

## Purpose

- Keep reusable Python viewer logic in one place
- Let firmware projects keep thin wrappers and launch scripts
- Make it safe to grow support for multiple sensors without duplicating rendering/runtime code

## Ownership Boundaries

Shared code belongs in `python_viewers/` when it is reusable across projects:

- `python_viewers/requirements.txt`
- `python_viewers/sensor_viewers/`
- shared scene, geometry, runtime, GUI, and host-side serial handling code

Project-local code stays in each sensor project:

- PlatformIO firmware
- sensor wiring details
- project launch scripts
- device-specific serial protocol unless a deliberate shared abstraction is introduced

## Layout

```
python_viewers/
├── requirements.txt              shared Python dependency set
├── sensor_viewers/
│   ├── ld06_viewer/              2D top-down viewer for LD06_lidar
│   └── vl53l5cx_viewer/          3D point cloud viewer for VL53L5CX_tof
```

## Package Contract

- Shared viewer modules live under `python_viewers/sensor_viewers/`
- Each sensor viewer: `sensor_viewers/<sensor_name>_viewer/`
- Project-local `viewer/` folders are wrappers/entrypoints only
- Shared dependencies belong in `python_viewers/requirements.txt`
- Reusable scene/rendering/runtime utilities belong here, not copied into each project

## Environment

```bash
cd python_viewers
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Testing Expectations

When changing shared Python viewer code:

- run `python -m compileall` as a syntax/import check
- confirm README run instructions still match reality
- validate both `VL53L5CX_tof` and `LD06_lidar` viewers independently

## Rules

- Prefer behavior-level reuse over copy-paste
- Do not silently migrate a project from local viewer logic to shared logic without updating docs
- Update `python_viewers/README.md` when shared layout or usage changes
- Update the consuming project README if the shared setup or launch flow changes
