# VL53L5CX ToF — Agent Guide

Agent and contributor guide for `VL53L5CX_tof/`.

## Project Boundaries

- `VL53L5CX_tof/src/` — ESP32 firmware (PlatformIO, Adafruit Feather ESP32 v2)
- `VL53L5CX_tof/viewer/` — thin compatibility wrapper; entrypoint only, no real logic
- `py_scripts/sensor_viewers/vl53l5cx_viewer/` — shared host-side viewer (real implementation)
- `VL53L5CX_tof/scripts/` — convenience scripts for virtualenv setup and viewer launch

## Firmware Protocol

JSON lines over USB serial at `460800` baud.

```json
{"t":"status","stage":"...","detail":"...","v":"0.2.0"}
{"t":"imu","ts_us":123,"accel_mps2":[0,0,9.8],"gyro_rads":[0,0,0],"temp_c":28.1,"imu_ok":true,"v":"0.2.0"}
{"t":"tof","seq":12,"ts_us":456,"distances":[...64...],"status":[...64...],"tof_ok":true,"v":"0.2.0"}
```

## Hardware

- Controller: Adafruit Feather ESP32 v2
- Sensors: VL53L5CX (8x8 ToF), MPU6050 (IMU)
- I2C: SDA = GPIO 22, SCL = GPIO 20
- VL53L5CX at `0x29`, MPU6050 at `0x68` or `0x69`
- Firmware retries missing sensors every 5 seconds and reports failure as JSON

## Viewer

Real implementation lives in `py_scripts/sensor_viewers/vl53l5cx_viewer/`. The `VL53L5CX_tof/viewer/` folder is a compatibility wrapper — its `requirements.txt` forwards to `../../py_scripts/requirements.txt`.

When changing the desktop viewer: edit the shared package, not the wrapper.

## Run the Viewer

```bash
# from VL53L5CX_tof/
./scripts/setup_viewer_env.sh      # once
./scripts/run_viewer.sh --port /dev/ttyACM0
```

The scripts use a `.venv/` created at `VL53L5CX_tof/.venv/`. The `run_viewer.sh` script invokes `python -m viewer` which resolves to `VL53L5CX_tof/viewer/__main__.py`.

## Rules

- Prefer editing `py_scripts/sensor_viewers/vl53l5cx_viewer/` for viewer changes
- Keep `VL53L5CX_tof/viewer/` as a thin wrapper; do not move real logic back into it
- Keep firmware serial protocol generation in `VL53L5CX_tof/src/sensor_streamer.cpp`
- Update `VL53L5CX_tof/README.md` if hardware wiring, protocol, or setup flow changes
