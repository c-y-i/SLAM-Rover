# Shared Python Scripts

Shared host-side Python workspace.

## Modules

- `sensor_viewers.ld06_viewer` — viser 2D top-down viewer for the LD06 LiDAR
- `sensor_viewers.vl53l5cx_viewer` — viser 3D point cloud viewer for the VL53L5CX + MPU6050
- `rover_tools.controller_teleop` — rover teleop + live viewer (+ optional `--slam`)
- `rover_tools.record` — telemetry recorder to JSONL
- `rover_tools.replay` — offline replay through SLAM

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

# Rover teleop + SLAM
python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam

# Rover record/replay
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl
python -m rover_tools.replay --file run1.jsonl --speed 2.0

# VL53L5CX viewer (from VL53L5CX_tof/ via convenience script)
./scripts/run_viewer.sh --port /dev/ttyACM0
```
