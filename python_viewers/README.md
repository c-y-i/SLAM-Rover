# Shared Python Viewers

Host-side viewer code shared across sensor projects in this workspace. One virtualenv, one dependency set, one scene runtime.

## Modules

- `sensor_viewers.ld06_viewer` — Viser-based 2D top-down viewer for the LD06 lidar
- `sensor_viewers.vl53l5cx_viewer` — Viser-based 3D point cloud viewer for the VL53L5CX + MPU6050

## Install

```bash
cd python_viewers
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run the LD06 Viewer

```bash
python -m sensor_viewers.ld06_viewer --port /dev/ttyUSB0
```

## Run the VL53L5CX Viewer

From `VL53L5CX_tof/` using the convenience scripts:

```bash
./scripts/run_viewer.sh --port /dev/ttyACM0
```

Or directly from `python_viewers/`:

```bash
python -m sensor_viewers.vl53l5cx_viewer --port /dev/ttyACM0
```
