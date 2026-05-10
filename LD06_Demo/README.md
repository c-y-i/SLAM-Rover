# LD06 Demo Firmware

This is an ESP32 firmware/demo app for the reusable [LD06_LiDAR](../LD06_LiDAR/) library. It reads LD06 scan frames, streams newline-delimited JSON over serial, optionally reads IMU data, and can serve a lightweight browser viewer directly from the ESP32.

Firmware runs on an Adafruit Feather ESP32 v2. The desktop viewer is a Python app using viser, shared in [../py_scripts/](../py_scripts/).

## What This Does

- Uses `LD06_LiDAR` to read LD06 UART packets and assemble scan frames
- Optionally reads MPU6050 or BNO085 IMU data over I2C
- Streams newline-delimited JSON packets at 460800 baud
- Serves a fallback web viewer from the ESP32 when Wi-Fi is configured
- Supports the host-side live 2D top-down viewer and map accumulation tools

## Hardware

- Adafruit Feather ESP32 v2
- LD06 LiDAR over UART
- Optional MPU6050 or BNO085 IMU over I2C

The default MPU6050 wiring is SDA to GPIO 22 and SCL to GPIO 20. IMU addresses are probed from `config.h`. If no IMU is found, IMU messages are not emitted and the viewer continues with LiDAR-only data.

## Firmware Setup

```bash
platformio run --target upload
platformio device monitor --baud 460800
```

`platformio.ini` depends on the sibling `../LD06_LiDAR` library via `symlink://../LD06_LiDAR`.

## Serial Protocol

JSON lines at 460800 baud.

```json
{"t":"scan","x":[...],"y":[...],"i":[...]}
```

`x` and `y` are millimeters. `i` is intensity from 0 to 255.

```json
{"t":"imu","accel_mps2":[x,y,z],"gyro_rads":[x,y,z],"ts_us":N}
```

Only emitted when an IMU is found.

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

## Run The Viewer

```bash
python -m sensor_viewers.ld06_viewer --port /dev/ttyUSB0
```

## Viewer Features

- Live 2D top-down scan in `/robot` frame
- Color modes: Radar, Distance, Intensity
- IMU-based orientation via Madgwick filter when IMU data is present
- Map accumulation with GUI clear button

## Web Viewer

The firmware also serves a lightweight 2D web viewer directly from the ESP32 through `src/web_viewer.*`. This is independent of the Python viewer.

## Notes

- Shared viewer code: [../py_scripts/sensor_viewers/ld06_viewer/](../py_scripts/sensor_viewers/ld06_viewer/)
