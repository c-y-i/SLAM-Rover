# SLAM Package

Pure-Python 2D LiDAR+IMU SLAM used by both rover teleop and the simulator.

## What's in `slam/`

- `slam_thread.py` — orchestration thread (IMU-assisted ICP + occupancy grid updates + viser publishing)
- `icp.py` — scan filtering, downsampling, and point-to-point ICP
- `occupancy_grid.py` — log-odds occupancy grid + Bresenham ray-casting
- `sim.py` — hardware-free simulator that feeds synthetic scans and IMU yaw into `SlamThread`

## Setup

Use the shared Python environment:

```bash
cd py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run Simulator (No Hardware)

From repo root:

```bash
python -m slam.sim --map office
```

You can also use the compatibility path:

```bash
python slam_sim/sim.py --map office
```

Common flags:

- `--web-port 8080` viewer port
- `--map random|office|warehouse|maze`
- `--seed 42` deterministic map when using `--map random`

Open `http://localhost:8080`.

## Run on Real Rover

Start rover teleop with SLAM:

```bash
cd py_scripts
source .venv/bin/activate
python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam
```

## Record and Replay

```bash
cd py_scripts
source .venv/bin/activate
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl
python -m rover_tools.replay --file run1.jsonl --speed 2.0
```
