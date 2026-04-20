# LD06 Lidar

Streams LD06 lidar scans (and optional MPU6050 IMU data) from an ESP32 over serial, then renders a live 2D top-down map in a desktop viewer.

The firmware runs on an Adafruit Feather ESP32 v2. The desktop viewer is a Python app built with viser, shared in [../python_viewers/](../python_viewers/).

## What This Does

- Reads LD06 scan data over UART
- Optionally reads MPU6050 accel + gyro over I2C
- Streams newline-delimited JSON packets at 460800 baud
- Hosts a lightweight web viewer on the ESP32 (fallback)
- Renders a live 2D top-down map with map accumulation on the desktop

## Hardware

### Controller

- Adafruit Feather ESP32 v2

### Sensors

- LD06 lidar (UART)
- MPU6050 (optional, I2C)

### I2C Wiring (MPU6050)

- `SDA -> GPIO 22`
- `SCL -> GPIO 20`
- `3V3 -> VCC`
- `GND -> GND`

MPU6050 probed at `0x68` then `0x69`. If not found, IMU messages are never emitted and the viewer handles this gracefully.

## Firmware Setup

```bash
platformio run --target upload
platformio device monitor --baud 460800
```

## Serial Protocol

JSON lines at 460800 baud.

### Scan Packet

```json
{"t":"scan","x":[...],"y":[...],"i":[...]}
```

`x`/`y` in mm, `i` = intensity 0–255.

### IMU Packet

```json
{"t":"imu","accel_mps2":[x,y,z],"gyro_rads":[x,y,z],"ts_us":N}
```

Only emitted if MPU6050 is found.

### Status Packet

```json
{"t":"status","stage":"...","detail":"..."}
```

## Viewer Setup

The viewer lives in the shared `python_viewers/` workspace.

```bash
cd ../python_viewers
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run the Viewer

```bash
python -m sensor_viewers.ld06_viewer --port /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` with your actual serial port.

## Viewer Features

- Live 2D top-down scan in `/robot` frame
- Color modes: Radar, Distance, Intensity
- IMU-based orientation via Madgwick filter
- Map accumulation with GUI clear button
- SLAM hooks: `update_pose(x, y, theta_rad)` / `get_latest_scan()`

## Web Viewer (Fallback)

The firmware also serves a lightweight 2D web viewer directly from the ESP32. See `src/web_viewer.h`. This is independent of the Python viewer.

## Notes

- Shared viewer code: [../python_viewers/sensor_viewers/ld06_viewer/](../python_viewers/sensor_viewers/ld06_viewer/)
- Agent guide: [../AGENTS/LD06_lidar.md](../AGENTS/LD06_lidar.md)
