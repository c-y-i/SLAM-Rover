# LD06 Mapping

LD06 Mapping is a LiDAR mapping workspace built around the LD06 2D LiDAR. It includes a reusable Arduino/PlatformIO LD06 library, ESP32 firmware for streaming scan data, host-side Python viewers and rover tools, and a pure-Python 2D SLAM simulator.

The rover pieces are still real subsystems in this workspace; `rover_stack/` keeps its existing name and firmware role.

## What's Here

| Folder | Description |
|---|---|
| [LD06_LiDAR/](LD06_LiDAR/) | Reusable Arduino/PlatformIO library for LD06 packet parsing and scan assembly |
| [LD06_Demo/](LD06_Demo/) | ESP32 PlatformIO firmware demo that uses `LD06_LiDAR`, streams JSON over serial, and serves an optional web viewer |
| [slam/](slam/) | Pure-Python 2D SLAM package and hardware-free simulator (`slam/sim.py`) |
| [py_scripts/](py_scripts/) | Shared host-side Python workspace for sensor viewers and rover tools |
| [rover_stack/](rover_stack/) | Rover firmware and tools |
| [VL53L5CX_tof/](VL53L5CX_tof/) | Standalone 8x8 ToF depth streaming firmware and viewer |

## Use The LD06 Library

The reusable library is in [LD06_LiDAR/](LD06_LiDAR/). The installable package name is `LD06_LiDAR`, and sketches include the same public header:

```cpp
#include <LD06_LiDAR.h>
```

PlatformIO users can clone this repository and point their firmware project at the `LD06_LiDAR/` folder:

```bash
git clone https://github.com/c-y-i/LD06-mapping.git
```

```ini
lib_deps =
  symlink:///path/to/LD06-mapping/LD06_LiDAR
```

For a sibling firmware project inside this repo, the relative form is:

```ini
lib_deps =
  symlink://../LD06_LiDAR
```

Arduino IDE users can download this repository as a ZIP, extract it, and copy only the `LD06_LiDAR/` folder into their Arduino `libraries/` folder. Restart the IDE, then open one of the examples from `LD06_LiDAR/examples/`.

## Try It Without Hardware

```bash
python -m slam.sim --map office
# open http://localhost:8080
```

Plot-only mode:

```bash
python -m slam.plot_sim
```

Static, non-live plot:

```bash
python -m slam.plot_sim --no-live
```

Plot on bundled static maps:

```bash
python -m slam.plot_sim --map nav2_tb3_sandbox
```

## Run The LD06 Firmware Demo

```bash
cd LD06_Demo
platformio run --target upload
platformio device monitor --baud 460800
```

The firmware reads LD06 UART packets through the reusable `LD06_LiDAR` library, emits newline-delimited JSON scan messages, optionally reads IMU data, and can serve a browser viewer from the ESP32.

## Run With The Rover

```bash
cd py_scripts
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam
```

Record and replay:

```bash
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl
python -m rover_tools.replay --file run1.jsonl [--speed 2.0]
```

## Flash Rover Firmware

```bash
cd rover_stack/controller && platformio run --target upload
cd rover_stack/bot        && platformio run --target upload
```

Set `kBotMac` in `rover_stack/controller/src/main.cpp` to your bot's MAC address.
