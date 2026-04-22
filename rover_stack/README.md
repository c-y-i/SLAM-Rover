# Rover Stack

ESP32-C3 robot-control suite with two PlatformIO projects plus host-side teleop tooling.

## Layout

```text
rover_stack/
├── controller/     USB bridge: PC serial <-> ESP-NOW <-> bot
├── bot/            robot firmware: motors + LD06 + telemetry forwarding
└── py/             keyboard teleop + web viewer tools
```

Firmware structure keeps orchestration in each `main.cpp`, with focused subsystems split into dedicated files (for example `bot/src/lidar_reader.*`, `bot/src/autonomy_controller.*`, `bot/src/motor_driver.*`, `bot/src/imu_support.*`, `bot/src/telemetry_link.*`, and `bot/src/status_led.*`).

## Current Data Flow

- Control: `PC -> Controller USB serial -> ESP-NOW -> Bot`
- Lidar visualization: `LD06 -> Bot -> ESP-NOW -> Controller -> USB serial -> Python teleop/viewer`

## Firmware

### `controller/`

- Parses USB commands (`1`, `2`, `x`, `w/a/s/d/q/e`, `Lf200`-style direct wheel commands)
- Sends bot commands over ESP-NOW
- Receives lidar telemetry chunks from bot
- Re-emits viewer-compatible JSON scan packets over USB serial at `460800`

### `bot/`

- Receives commands over ESP-NOW
- Supports mode switching (`1` wall-follow autonomy, `2` teleop, `x` stop)
- Uses a generic motor-driver layer with compile-time profiles (`TB6612` default, `L298` optional)
- Applies emergency obstacle override in all drive modes
- Reads LD06 over UART and forwards downsampled scan telemetry to controller
- Integrates optional BNO085 IMU and forwards IMU telemetry through controller

## Build

```bash
cd controller
platformio run
```

```bash
cd ../bot
platformio run
```

`bot/platformio.ini` defaults to `MOTOR_DRIVER_TYPE=1` (`TB6612`). Set `MOTOR_DRIVER_TYPE=0` for `L298`.

## Run Host Teleop + Viewer

```bash
cd py
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080
```

Open `http://localhost:8080` while using keyboard teleop in the terminal.

## SLAM (optional)

Add `--slam` to enable pure-Python 2D LiDAR+IMU SLAM on the host:

```bash
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam
```

The SLAM pipeline (`py/slam/`) runs in a background thread:

- IMU yaw from the BNO085 seeds each ICP iteration as a rotation prior.
- Point-to-point ICP (scipy KDTree + SVD) refines translation between consecutive scans.
- An occupancy grid is built via Bresenham ray-casting and rendered in the viser web view as `/slam/grid`.
- The robot frame in the web viewer moves with the estimated pose.
- Without `--slam` the teleop behaves identically to before.

## Notes

- `controller/src/main.cpp` currently uses a fixed `kBotMac` address; update it for your bot board.
- `py/controller_teleop.py` now shows IMU health, heading, and turn-rate when IMU packets are available.
