# SLAM Package

Pure-Python 2D LiDAR+IMU SLAM used by both rover teleop and the simulator.

## What's in `slam/`

- `slam_thread.py` — orchestration thread (IMU-assisted ICP + occupancy grid updates + viser publishing)
- `icp.py` — scan filtering, downsampling, and point-to-point ICP
- `occupancy_grid.py` — log-odds occupancy grid + Bresenham ray-casting
- `sim.py` — hardware-free simulator that feeds synthetic scans and IMU yaw into `SlamThread`
- `plot_sim.py` — Matplotlib simulator/diagnostic plotter (no browser UI)

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

Common flags:

- `--web-port 8080` viewer port
- `--map random|office|warehouse|maze|nav2_tb3_sandbox|nav2_warehouse|nav2_depot`
- `--seed 42` deterministic map when using `--map random`

Open `http://localhost:8080`.

## Run Plot-Only Simulator (No Browser)

From repo root:

```bash
python -m slam.plot_sim
```

Defaults for `python -m slam.plot_sim`:

- map = `random`
- live GUI updates = enabled
- final plot auto-saved under `artifacts/plots/` (gitignored)
- only latest 5 final plots are retained

Useful options:

- `--map office|warehouse|maze|random` generated sim worlds
- `--map nav2_tb3_sandbox|nav2_warehouse|nav2_depot` static `.yaml/.pgm` maps bundled in `slam/static_maps/nav2/`
- `--no-live` disable live updates and render only the final plot
- `--update-every 10` redraw interval for `--live`
- `--steps 2000` longer run
- `--speed 1.0` slower motion (usually lower drift)
- `--range-noise 0.01 --yaw-noise 0.2` lower-noise stress test
- `--save slam_plot.png --no-show` export image in headless sessions
- `--no-save-repo` disable default auto-save to `artifacts/plots/`
- `--seed 42` deterministic generated map/route seed

Example static map run:

```bash
python -m slam.plot_sim --map nav2_tb3_sandbox
```

The plot includes:
- occupancy map
- ground-truth vs estimated trajectory
- position and heading error over scan index

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
