# Rover Stack — Agent Guide

Agent and contributor guide for `rover_stack/`.

## Project Boundaries

- `rover_stack/controller/` — ESP32-C3 USB bridge firmware (PlatformIO)
- `rover_stack/bot/` — ESP32-C3 robot firmware (PlatformIO)
- `rover_stack/py/` — suite-local teleop + web viewer integration
- `py_scripts/` — shared viewer/runtime code reused across projects

Keep robot/controller protocol and behavior in this suite. Keep generic reusable viewer/runtime code in `py_scripts/`.

## Firmware Protocol Roles

- `controller/`:
  - accepts PC serial commands
  - sends commands to bot over ESP-NOW
  - receives bot lidar + IMU telemetry
  - emits JSON lines over USB for host tools (`scan`, `imu`, `status`)
- `bot/`:
  - receives mode + motion commands over ESP-NOW
  - drives differential motors via compile-time driver profile
  - reads LD06 data and optional BNO085 IMU
  - runs wall-follow + obstacle avoidance
  - forwards telemetry to controller

Target topology:

- Control: `PC -> Controller USB serial -> ESP-NOW -> Bot`
- Telemetry: `Bot sensors -> ESP-NOW -> Controller -> USB serial -> Host tools`

## Code Layout

```text
rover_stack/
├── README.md
├── controller/
│   ├── platformio.ini
│   └── src/
│       └── main.cpp
├── bot/
│   ├── platformio.ini
│   └── src/
│       ├── main.cpp
│       ├── lidar_data.h
│       ├── lidar_reader.h
│       ├── lidar_reader.cpp
│       ├── autonomy_controller.h
│       ├── autonomy_controller.cpp
│       ├── imu_support.h
│       ├── imu_support.cpp
│       ├── motor_driver.h
│       ├── motor_driver.cpp
│       ├── telemetry_link.h
│       ├── telemetry_link.cpp
│       ├── status_led.h
│       └── status_led.cpp
└── py/
    ├── README.md
    ├── requirements.txt
    └── controller_teleop.py
```

Structure rule for this suite:

- Keep high-level behavior/orchestration in each firmware `main.cpp`
- Split focused subsystems into dedicated files (e.g., LiDAR parsing, autonomy stack, motor-driver layer)
- Avoid thin-wrapper `*_app.cpp` indirection for this project

## Build

```bash
cd rover_stack/controller
platformio run
```

```bash
cd rover_stack/bot
platformio run
```

## Host Tooling

- `rover_stack/py/controller_teleop.py` combines local keyboard teleop and web visualization in one process.
- TA_Bot reference source at `Arduino/5100/TA_Bot/` is the protocol/behavior baseline.

## Rules

- Keep command surface compatible with TA_Bot (`1`, `2`, `x`, `w/a/s/d/q/e`, `Lf200`-style direct commands)
- Keep controller USB output JSON-line friendly and viewer-compatible
- Keep bot motor drivers generic and selectable with `MOTOR_DRIVER_TYPE` (`TB6612` default, `L298` alternate)
- Keep mode `1` autonomy deterministic (wall-follow + avoidance) and keep emergency override active in all modes
- Keep IMU optional with LiDAR-only fallback when IMU is unavailable
- Update `rover_stack/README.md` when protocol, wiring assumptions, or host workflow changes
