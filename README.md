# embedded-toolkit

A scratchpad for embedded projects - sensors, cameras, motors, flight controllers, MCUs & SBSc, etc. Firmware lives alongside host-side tooling where relevant.

> I still have lots of code pieces all over the place, need some time to properly organize and put them here.

## Sensors

| Folder | Hardware | What it does |
|---|---|---|
| [LD06_lidar/](LD06_lidar/) | LD06 lidar + optional IMU | Streams 2D scan + IMU over serial, renders top-down live map |
| [VL53L5CX_tof/](VL53L5CX_tof/) | VL53L5CX ToF + IMU | Streams 8x8 depth + IMU over serial, renders live 3D point cloud |

## Cameras


## Motors / Actuators

| Folder | Hardware | What it does |
|---|---|---|
| [rover_stack/](rover_stack/) | ESP32-C3 controller + ESP32-C3 bot | ESP-NOW control bridge + bot firmware + host teleop/web viewer |

## Flight Controllers


## Shared Python Scripts

[py_scripts/](py_scripts/) holds host-side Python code shared across projects — viewers plus future tooling.

```bash
cd py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## AI Agent Guidance

Agent guides live in [AGENTS/](AGENTS/). (made this for those of you vibe coders)

- [AGENTS/py_scripts.md](AGENTS/py_scripts.md) — shared Python workspace rules
- [AGENTS/LD06_lidar.md](AGENTS/LD06_lidar.md) — LD06 firmware + viewer agent guide
- [AGENTS/VL53L5CX_tof.md](AGENTS/VL53L5CX_tof.md) — VL53L5CX firmware + viewer agent guide
- [AGENTS/rover_stack.md](AGENTS/rover_stack.md) — rover stack suite agent guide
