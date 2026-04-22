# Shared Python Scripts

Host-side Python code shared across embedded-toolkit projects. Today this is mostly viewers, with room for future host tools.

## Modules

- `sensor_viewers.ld06_viewer` — Viser-based 2D top-down viewer for the LD06 lidar
- `sensor_viewers.vl53l5cx_viewer` — Viser-based 3D point cloud viewer for the VL53L5CX + MPU6050

## Install

```bash
cd py_scripts
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

Or directly from `py_scripts/`:

```bash
python -m sensor_viewers.vl53l5cx_viewer --port /dev/ttyACM0
```
