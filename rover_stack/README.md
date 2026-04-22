# Rover Stack

ESP32-C3 robot-control suite — two PlatformIO firmware projects and host-side tooling.

## Layout

```text
rover_stack/
├── controller/     USB bridge: PC serial <-> ESP-NOW <-> bot
├── bot/            robot firmware: motors + LD06 + BNO085 + telemetry
└── py/             host tools: teleop, web viewer, SLAM, record, replay
```

## Data Flow

- Control:   `PC → Controller USB serial → ESP-NOW → Bot`
- Telemetry: `Bot sensors → ESP-NOW → Controller → USB serial → Host tools`

## Firmware

### `controller/`

- Parses USB commands (`1`, `2`, `x`, `w/a/s/d/q/e`, `Lf200`-style direct wheel commands)
- Forwards commands to bot over ESP-NOW
- Receives LiDAR + IMU telemetry from bot and re-emits JSON lines over USB at 460800 baud

### `bot/`

- Receives commands over ESP-NOW
- Mode switching: `1` wall-follow autonomy, `2` teleop, `x` stop
- Motor-driver layer with compile-time profiles (`TB6612` default, `L298` optional)
- Emergency obstacle override active in all drive modes
- Reads LD06 over UART and forwards downsampled scan telemetry
- Reads BNO085 IMU and forwards heading + turn-rate telemetry

## Build

```bash
cd controller && platformio run
cd ../bot     && platformio run
```

`bot/platformio.ini` defaults to `MOTOR_DRIVER_TYPE=1` (TB6612). Set `MOTOR_DRIVER_TYPE=0` for L298.

Update `kBotMac` in `controller/src/main.cpp` to match your bot board's MAC address.

## Host Tools

All tools live in `py/`. One-time setup:

```bash
cd py
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Teleop + viewer

```bash
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080
```

Open `http://localhost:8080`. Keyboard controls active in the terminal.

### Teleop + SLAM

```bash
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam
```

Runs the SLAM stack (`slam/`) in a background thread:

- BNO085 absolute yaw seeds each ICP iteration as a rotation prior
- Point-to-point ICP (scipy KDTree + SVD) estimates translation between consecutive scans
- Occupancy grid built via Bresenham ray-casting, rendered in the viewer as `/slam/grid`
- Robot frame in the web viewer tracks the estimated pose
- Without `--slam` the teleop is unchanged

### Record a session

```bash
python record.py --port /dev/ttyACM0 [--baud 460800] [--output run1.jsonl]
```

Writes timestamped JSONL — every JSON line from the controller gets a `wall_s` field prepended.

### Replay offline

```bash
python replay.py --file run1.jsonl [--web-port 8080] [--speed 2.0]
# --speed 0 = as fast as possible
```

Feeds the recording through the full SLAM stack without any hardware. Keeps the viser viewer alive after replay ends for map inspection.
