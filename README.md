# SLAM Rover

Rover with LD06 LiDAR + BNO085 IMU, with 2D SLAM running on a host PC. Firmware written for ESP32 via PlatformIO. Pure Python for now, ROS2 support to be added.

> So far just deploying this on small platforms like mechatronics course TA bot, and drones for indoor mapping.

## What's here

### Rover

| Folder | Description |
|---|---|
| [rover_stack/](rover_stack/) | Rover firmware (ESP32, PlatformIO) and host Python tools |
| [slam/](slam/) | Pure-Python SLAM package + hardware-free simulator (`slam/sim.py`) |
| [slam_sim/](slam_sim/) | Compatibility launcher for the simulator (legacy path) |

### Standalone sensor subsystems

Each can be used on its own, independent of the rover.

| Folder | Hardware | Description |
|---|---|---|
| [LD06_lidar/](LD06_lidar/) | Feather ESP32 v2 + LD06 | 2D LiDAR streaming + live top-down viewer |
| [VL53L5CX_tof/](VL53L5CX_tof/) | Feather ESP32 v2 + VL53L5CX + MPU6050 | 8×8 ToF depth streaming + live 3D point cloud viewer |
| [py_scripts/](py_scripts/) | — | Shared host-side Python workspace (sensor viewers + rover tools) |

## Try it without hardware

```bash
python -m slam.sim --map office
# open http://localhost:8080
```

## Run with the rover

```bash
cd py_scripts
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt

python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam
```

Record and replay:

```bash
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl
python -m rover_tools.replay --file run1.jsonl [--speed 2.0]
```

## Flash firmware

```bash
cd rover_stack/controller && platformio run --target upload
cd rover_stack/bot        && platformio run --target upload
```

Set `kBotMac` in `controller/src/main.cpp` to your bot's MAC address.

## Agent guides (for you vibe coders...)

- [AGENTS/rover_stack.md](AGENTS/rover_stack.md)
- [AGENTS/slam.md](AGENTS/slam.md)
- [AGENTS/py_scripts.md](AGENTS/py_scripts.md)
