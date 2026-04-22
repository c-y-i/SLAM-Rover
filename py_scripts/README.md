# Shared Python Scripts

Host-side Python viewer code shared across the standalone sensor subsystems (LD06, VL53L5CX).

## Modules

- `sensor_viewers.ld06_viewer` — viser 2D top-down viewer for the LD06 LiDAR
- `sensor_viewers.vl53l5cx_viewer` — viser 3D point cloud viewer for the VL53L5CX + MPU6050

## Install

```bash
cd py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
# LD06 viewer
python -m sensor_viewers.ld06_viewer --port /dev/ttyUSB0

# VL53L5CX viewer (from py_scripts/)
python -m sensor_viewers.vl53l5cx_viewer --port /dev/ttyACM0

# VL53L5CX viewer (from VL53L5CX_tof/ via convenience script)
./scripts/run_viewer.sh --port /dev/ttyACM0
```
