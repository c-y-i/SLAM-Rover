# LD06 Lidar — Agent Guide

Agent and contributor guide for `LD06_lidar/`.

## Project Boundaries

- `LD06_lidar/src/` — ESP32 firmware (PlatformIO, Adafruit Feather ESP32 v2)
- `python_viewers/sensor_viewers/ld06_viewer/` — shared host-side viewer
- `LD06_lidar/` has no project-local viewer wrapper; run directly from `python_viewers/`

The ESP32 also serves a lightweight web viewer via `web_viewer.h`. That is a separate fallback; do not conflate it with the Python viser viewer.

## Firmware Protocol

JSON lines over USB serial at `460800` baud. Three message types:

```json
{"t":"scan","x":[...],"y":[...],"i":[...]}
```
`x`/`y` in mm, `i` = intensity 0–255.

```json
{"t":"imu","accel_mps2":[x,y,z],"gyro_rads":[x,y,z],"ts_us":N}
```
Optional — only emitted if MPU6050 is found on I2C.

```json
{"t":"status","stage":"...","detail":"..."}
```
Heartbeat and lifecycle events.

## Hardware

- Controller: Adafruit Feather ESP32 v2
- Sensors: LD06 lidar, optional MPU6050
- I2C: SDA = GPIO 22, SCL = GPIO 20
- MPU6050 probed at `0x68` then `0x69`; missing IMU is handled gracefully

## Viewer Features

- Live 2D top-down scan in `/robot` frame
- Color modes: Radar, Distance, Intensity
- IMU orientation via Madgwick filter (reused from `vl53l5cx_viewer.imu_fusion`)
- Map accumulation: world-frame deque, cleared by GUI button
- SLAM hooks: `update_pose(x, y, theta_rad)` and `get_latest_scan()` on `LD06Viewer`

## Run the Viewer

```bash
cd python_viewers
source .venv/bin/activate
python -m sensor_viewers.ld06_viewer --port /dev/ttyUSB0
```

## Rules

- Prefer editing `python_viewers/sensor_viewers/ld06_viewer/` for viewer changes
- Keep firmware-specific serial protocol generation in `LD06_lidar/src/`
- The web viewer (`web_viewer.h`) is firmware-only; do not merge it with the Python viewer
- Update `LD06_lidar/README.md` if hardware wiring or protocol changes
