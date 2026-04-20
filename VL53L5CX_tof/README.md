# VL53L5CX + MPU6050 Viewer

This project streams `VL53L5CX` time-of-flight data and `MPU6050` IMU data from an ESP32 over serial, then renders the live point cloud in a desktop 3D viewer.

The firmware runs on an `Adafruit Feather ESP32 V2`. The desktop viewer is a Python app built with `viser`.

## What This Does

- Reads `VL53L5CX` ranging data in `8x8` mode over I2C
- Reads `MPU6050` accel + gyro data over the same I2C bus
- Streams newline-delimited JSON packets at `460800` baud
- Runs host-side 6-DoF IMU fusion
- Renders the live point cloud, zone rays, and rig pose in a browser

## Hardware

### Controller

- `Adafruit Feather ESP32 V2`

### Sensors

- `VL53L5CX`
- `MPU6050`

### I2C Wiring

Both sensors share the same I2C bus.

- `SDA -> GPIO 22`
- `SCL -> GPIO 20`
- `3V3 -> sensor VCC`
- `GND -> sensor GND`

Expected I2C addresses:

- `VL53L5CX -> 0x29`
- `MPU6050 -> 0x68` or `0x69`

## Project Layout

- `src/`
  ESP32 firmware that probes sensors and streams serial JSON
- `viewer/`
  Thin compatibility wrapper so this project can still launch the shared viewer with `python -m viewer`
- `../python_viewers/`
  Shared host-side Python viewer code for this workspace
- `scripts/setup_viewer_env.sh`
  Creates a local virtual environment and installs viewer dependencies
- `scripts/run_viewer.sh`
  Runs the viewer using the local virtual environment

## Firmware Setup

From the project root:

```bash
platformio run --target upload
platformio device monitor --baud 460800
```

The firmware will emit status packets during startup, for example:

```json
{"t":"status","stage":"boot","detail":"starting","v":"0.2.0"}
{"t":"status","stage":"i2c_scan","detail":"found_0x29_0x68","v":"0.2.0"}
{"t":"status","stage":"tof","detail":"ready","v":"0.2.0"}
{"t":"status","stage":"imu","detail":"ready_0x68","v":"0.2.0"}
```

If a device is missing, the firmware keeps retrying every 5 seconds and reports the failure as JSON.

## Viewer Setup

### Recommended: local virtual environment

Debian and Ubuntu now block `pip install` into the system Python by default. This repo is set up to avoid that.

From the project root:

```bash
./scripts/setup_viewer_env.sh
```

That script will:

- create `.venv/`
- upgrade `pip`
- install `viewer/requirements.txt`

Then start the viewer with:

```bash
./scripts/run_viewer.sh --port /dev/ttyACM0
```

If your board enumerates differently, replace `/dev/ttyACM0` with your actual serial port.

### Manual virtualenv setup

If you prefer doing it yourself:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r viewer/requirements.txt
python -m viewer --port /dev/ttyACM0
```

If you `cd viewer/` first, use:

```bash
pip install -r requirements.txt
```

not `pip install -r viewer/requirements.txt`.

## Viewer Controls

The viewer opens a local web UI served by `viser`, usually at:

```text
http://localhost:8080
```

Available controls:

- point size
- zone ray visibility
- IMU rotation enable/disable

Displayed live status:

- I2C scan result
- ToF connected/disconnected
- ToF status detail
- IMU connected/disconnected
- IMU status detail
- ToF FPS
- IMU FPS
- current distance range

## Serial Protocol

The firmware emits three packet types.

### Status Packet

```json
{"t":"status","stage":"...","detail":"...","v":"0.2.0"}
```

### IMU Packet

```json
{"t":"imu","ts_us":123,"accel_mps2":[0,0,9.8],"gyro_rads":[0,0,0],"temp_c":28.1,"imu_ok":true,"v":"0.2.0"}
```

### ToF Packet

```json
{"t":"tof","seq":12,"ts_us":456,"distances":[...64...],"status":[...64...],"tof_ok":true,"v":"0.2.0"}
```

## Typical Workflow

1. Wire the sensors to the shared I2C bus.
2. Upload the firmware with PlatformIO.
3. Open the serial monitor and confirm the `i2c_scan`, `tof`, and `imu` status packets look healthy.
4. Run `./scripts/setup_viewer_env.sh` once.
5. Run `./scripts/run_viewer.sh --port /dev/ttyACM0`.
6. Open the viewer URL in your browser.

## Troubleshooting

### `externally-managed-environment`

Do not use system `pip` directly. Use:

```bash
./scripts/setup_viewer_env.sh
```

or create a virtualenv manually.

### `i2c_scan` reports `none`

- check `3V3`
- check `GND`
- check `SDA` on GPIO `22`
- check `SCL` on GPIO `20`
- make sure SDA/SCL are not swapped

### Only one sensor appears on I2C

- `VL53L5CX` should answer on `0x29`
- `MPU6050` should answer on `0x68` or `0x69`

### IMU yaw drifts over time

This is expected in this version. `MPU6050` is a 6-DoF IMU with no magnetometer, so yaw will drift over time.

## Notes

- Viewer dependencies: [viewer/requirements.txt](viewer/requirements.txt) (forwards to `../python_viewers/requirements.txt`)
- Shared viewer code: [../python_viewers/sensor_viewers/vl53l5cx_viewer/](../python_viewers/sensor_viewers/vl53l5cx_viewer/)
- Agent guide: [../AGENTS/VL53L5CX_tof.md](../AGENTS/VL53L5CX_tof.md)
- Scene and pose defaults: [../python_viewers/sensor_viewers/vl53l5cx_viewer/config.py](../python_viewers/sensor_viewers/vl53l5cx_viewer/config.py)
- Firmware protocol: [src/sensor_streamer.cpp](src/sensor_streamer.cpp)
