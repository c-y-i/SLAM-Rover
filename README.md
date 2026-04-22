# SLAM Rover

A rover you can actually deploy. ESP32-C3 bot + controller, LD06 LiDAR, BNO085 IMU, and a pure-Python SLAM stack that runs on any laptop — no ROS, no Docker, no drama.

Point it at a wall, watch it map the room, drive it with a keyboard. That's the idea.

## What's in the box

| Component | Where |
|---|---|
| Bot + controller firmware (ESP32-C3, ESP-NOW) | [rover_stack/](rover_stack/) |
| Pure-Python SLAM (ICP + occupancy grid) | [slam/](slam/) |
| Hardware-free SLAM simulator | [slam_sim/](slam_sim/) |
| LD06 LiDAR standalone firmware + viewer | [LD06_lidar/](LD06_lidar/) |
| VL53L5CX ToF standalone firmware + viewer | [VL53L5CX_tof/](VL53L5CX_tof/) |
| Shared Python viewer code | [py_scripts/](py_scripts/) |

## Try it now (no hardware needed)

```bash
python slam_sim/sim.py --map office
```

Open `http://localhost:8080`. Watch a simulated rover navigate and build a map in real time. Switch maps, add noise, pause and resume — all from the browser.

## Run on real hardware

```bash
cd rover_stack/py
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt

# Drive it and map it
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam
```

Record a run, replay it later:

```bash
python record.py  --port /dev/ttyACM0 --output run1.jsonl
python replay.py  --file run1.jsonl --speed 0   # as fast as possible
```

## Flash the firmware

```bash
cd rover_stack/controller && platformio run --target upload
cd rover_stack/bot        && platformio run --target upload
```

Set `kBotMac` in `controller/src/main.cpp` to your bot board's MAC address before flashing.

## Goals

- Drop-in SLAM for any differential-drive robot with a 2D LiDAR and an absolute-heading IMU
- No native extensions — pure numpy/scipy, runs wherever Python runs
- Modular: swap the hardware bridge, keep the SLAM stack unchanged

## Agent guides

Working with an AI coding assistant? Start here:

- [AGENTS/rover_stack.md](AGENTS/rover_stack.md) — firmware + host tooling
- [AGENTS/slam.md](AGENTS/slam.md) — SLAM algorithm, sim, recording
- [AGENTS/py_scripts.md](AGENTS/py_scripts.md) — shared Python viewer workspace
