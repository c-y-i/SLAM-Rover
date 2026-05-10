# Example Rover Integration

Example ESP32 robot integration for the `LD06_LiDAR` library. It shows one way to put the LD06 on a small rover, stream scan telemetry over a controller bridge, and use host-side tools for teleop and viewing.

## Layout

```text
rover_stack/
├── controller/     example USB bridge: PC serial <-> ESP-NOW <-> bot
└── bot/            example robot firmware: motors + LD06 + BNO085 + telemetry
```

## Data Flow

- Control:   `PC → Controller USB serial → ESP-NOW → Bot`
- Telemetry: `Bot sensors → ESP-NOW → Controller → USB serial → Host tools`

## Example Firmware

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

## Host Tools For The Example

Host tools live in the repo-level `py_scripts/rover_tools/` package.

One-time setup:

```bash
cd ../py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Teleop + viewer

```bash
python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080
```

Open `http://localhost:8080`. Keyboard controls active in the terminal.

### Record a session

```bash
python -m rover_tools.record --port /dev/ttyACM0 [--baud 460800] [--output run1.jsonl]
```

Writes timestamped JSONL — every JSON line from the controller gets a `wall_s` field prepended.

Run host tools from `../py_scripts`.
