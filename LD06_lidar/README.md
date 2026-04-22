# LD06 Lidar

Streams LD06 lidar scans (and optional MPU6050 IMU data) from an ESP32 over serial, then renders a live 2D top-down map in a desktop viewer.

Firmware runs on an Adafruit Feather ESP32 v2. Desktop viewer is a Python app using viser, shared in [../py_scripts/](../py_scripts/).

## What This Does

- Reads LD06 scan data over UART
- Optionally reads MPU6050 accel + gyro over I2C
- Streams newline-delimited JSON packets at 460800 baud
- Renders a live 2D top-down map with map accumulation on the desktop

## Hardware

- Adafruit Feather ESP32 v2
- LD06 lidar (UART)
- MPU6050 (optional, I2C): SDA → GPIO 22, SCL → GPIO 20, probed at `0x68` then `0x69`

If MPU6050 is not found, IMU messages are never emitted; the viewer handles this gracefully.

## Firmware Setup

```bash
platformio run --target upload
platformio device monitor --baud 460800
```

## Serial Protocol

JSON lines at 460800 baud.

```json
{"t":"scan","x":[...],"y":[...],"i":[...]}
```
`x`/`y` in mm, `i` = intensity 0–255.

```json
{"t":"imu","accel_mps2":[x,y,z],"gyro_rads":[x,y,z],"ts_us":N}
```
Only emitted if MPU6050 is found.

```json
{"t":"status","stage":"...","detail":"..."}
```

## Viewer Setup

```bash
cd ../py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run the Viewer

```bash
python -m sensor_viewers.ld06_viewer --port /dev/ttyUSB0
```

## Viewer Features

- Live 2D top-down scan in `/robot` frame
- Color modes: Radar, Distance, Intensity
- IMU-based orientation via Madgwick filter
- Map accumulation with GUI clear button

## Web Viewer (Firmware Fallback)

The firmware also serves a lightweight 2D web viewer directly from the ESP32 (`src/web_viewer.h`). This is independent of the Python viewer.

## Notes

- Shared viewer code: [../py_scripts/sensor_viewers/ld06_viewer/](../py_scripts/sensor_viewers/ld06_viewer/)
- Agent guide: [../AGENTS/LD06_lidar.md](../AGENTS/LD06_lidar.md)
