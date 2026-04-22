# rover_stack Python tools

Suite-local host tooling for the rover stack workflow.

## Included tool

- `controller_teleop.py` — single-process bridge that combines:
  - keyboard teleop (`1`, `2`, `x`, `w/a/s/d/q/e`, direct `Lf200` commands)
  - controller USB serial I/O
  - live web lidar viewer (viser)
  - IMU health + heading/turn-rate display when IMU telemetry is present

## Setup

```bash
cd py
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080
```

Then open `http://localhost:8080`.

The TA_Bot reference script remains at `../../../Arduino/5100/TA_Bot/scripts/controller_teleop.py`.
