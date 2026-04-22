# VL53L5CX + MPU6050 Viewer

Streams VL53L5CX time-of-flight (8×8) and MPU6050 IMU data from an ESP32 over serial, then renders a live 3D point cloud in a desktop viewer.

Firmware runs on an Adafruit Feather ESP32 v2. Desktop viewer is built with viser, shared in [../py_scripts/](../py_scripts/).

## What This Does

- Reads VL53L5CX ranging data in 8×8 mode over I2C
- Reads MPU6050 accel + gyro over the same I2C bus
- Streams newline-delimited JSON packets at 460800 baud
- Runs host-side 6-DoF IMU fusion (Madgwick)
- Renders live point cloud, zone rays, and rig pose in a browser

## Hardware

- Adafruit Feather ESP32 v2
- VL53L5CX (I2C at `0x29`)
- MPU6050 (I2C at `0x68` or `0x69`)
- Shared I2C bus: SDA → GPIO 22, SCL → GPIO 20

Firmware retries missing sensors every 5 seconds and reports failure as JSON.

## Firmware Setup

```bash
platformio run --target upload
platformio device monitor --baud 460800
```

Startup emits status packets:

```json
{"t":"status","stage":"boot","detail":"starting","v":"0.2.0"}
{"t":"status","stage":"i2c_scan","detail":"found_0x29_0x68","v":"0.2.0"}
{"t":"status","stage":"tof","detail":"ready","v":"0.2.0"}
{"t":"status","stage":"imu","detail":"ready_0x68","v":"0.2.0"}
```

## Viewer Setup

```bash
./scripts/setup_viewer_env.sh   # creates .venv/ and installs deps — run once
./scripts/run_viewer.sh --port /dev/ttyACM0
```

Or manually:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r viewer/requirements.txt
python -m viewer --port /dev/ttyACM0
```

## Viewer Controls

Open `http://localhost:8080`. GUI controls: point size, zone ray visibility, IMU rotation toggle.
Live status panel shows I2C scan result, ToF/IMU connected state, FPS, and distance range.

## Serial Protocol

```json
{"t":"status","stage":"...","detail":"...","v":"0.2.0"}
{"t":"imu","ts_us":123,"accel_mps2":[0,0,9.8],"gyro_rads":[0,0,0],"temp_c":28.1,"imu_ok":true,"v":"0.2.0"}
{"t":"tof","seq":12,"ts_us":456,"distances":[...64...],"status":[...64...],"tof_ok":true,"v":"0.2.0"}
```

## Troubleshooting

**`externally-managed-environment`** — use `./scripts/setup_viewer_env.sh`, not system pip.

**`i2c_scan` reports `none`** — check 3V3, GND, SDA on GPIO 22, SCL on GPIO 20, and that SDA/SCL aren't swapped.

**IMU yaw drifts** — expected; MPU6050 has no magnetometer.

## Notes

- Shared viewer code: [../py_scripts/sensor_viewers/vl53l5cx_viewer/](../py_scripts/sensor_viewers/vl53l5cx_viewer/)
- `viewer/` folder here is a thin wrapper entrypoint only — edit the shared package for real changes
- Agent guide: [../AGENTS/VL53L5CX_tof.md](../AGENTS/VL53L5CX_tof.md)
