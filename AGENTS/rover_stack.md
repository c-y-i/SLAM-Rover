# Rover Stack — Agent Guide

Agent and contributor guide for `rover_stack/`.

## Project Boundaries

- `rover_stack/controller/` — USB bridge firmware (PlatformIO, ESP32)
- `rover_stack/bot/` — robot firmware (PlatformIO, ESP32)
- `rover_stack/py/` — compatibility launchers for host tools
- `slam/` — standalone SLAM package at repo root (imported by py tools and simulator)
- `py_scripts/` — shared Python workspace, including rover tools in `py_scripts/rover_tools/`

Keep robot/controller protocol and behavior in `rover_stack/`. Keep the SLAM algorithm in `slam/`. Keep generic reusable viewer code in `py_scripts/`.

## Firmware Protocol Roles

- `controller/`:
  - accepts PC serial commands
  - sends commands to bot over ESP-NOW
  - receives bot LiDAR + IMU telemetry
  - emits JSON lines over USB: `scan`, `imu`, `status`
- `bot/`:
  - receives mode + motion commands over ESP-NOW
  - drives differential motors via compile-time driver profile
  - reads LD06 over UART, BNO085 over I2C
  - runs wall-follow + obstacle avoidance
  - forwards telemetry to controller

Target topology:

- Control:   `PC → Controller USB serial → ESP-NOW → Bot`
- Telemetry: `Bot sensors → ESP-NOW → Controller → USB serial → Host tools`

## Code Layout

```text
rover_stack/
├── README.md
├── controller/
│   ├── platformio.ini
│   └── src/main.cpp
├── bot/
│   ├── platformio.ini
│   └── src/
│       ├── main.cpp
│       ├── lidar_data.h / lidar_reader.*
│       ├── autonomy_controller.*
│       ├── motor_driver.*
│       ├── imu_support.*
│       ├── telemetry_link.*
│       └── status_led.*
└── py/
    ├── README.md
    ├── requirements.txt       ← symlink to ../../py_scripts/requirements.txt
    ├── controller_teleop.py   ← compatibility launcher
    ├── record.py              ← compatibility launcher
    └── replay.py              ← compatibility launcher

py_scripts/
└── rover_tools/
    ├── controller_teleop.py   ← teleop + viewer + optional SLAM
    ├── record.py              ← records JSONL from controller USB
    └── replay.py              ← replays JSONL through SLAM offline

slam/                          ← repo root, shared by rover tools and simulator
├── __init__.py
├── icp.py
├── occupancy_grid.py
├── slam_thread.py
└── sim.py
```

## Build

```bash
cd rover_stack/controller && platformio run
cd rover_stack/bot        && platformio run
```

## Host Tooling

```bash
cd py_scripts
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt

python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080 [--slam]
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl
python -m rover_tools.replay --file run1.jsonl [--speed 2.0]
```

## Rules

- Keep command surface compatible with TA_Bot (`1`, `2`, `x`, `w/a/s/d/q/e`, `Lf200`-style)
- Keep controller USB output as JSON lines — one packet type per line
- Keep bot motor drivers generic and selectable with `MOTOR_DRIVER_TYPE`
- Keep mode `1` autonomy deterministic (wall-follow + avoidance) with emergency override in all modes
- Keep IMU optional — LiDAR-only fallback when BNO085 is unavailable
- SLAM lives in `slam/` at repo root — do not nest it inside rover launcher folders
- `SlamThread` is duck-typed; the real bridge is `ControllerTeleopBridge.get_snapshot()`
- Update `rover_stack/README.md` when protocol, wiring, or host workflow changes
