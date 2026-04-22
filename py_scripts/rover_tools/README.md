# Rover Tools

Host-side tooling for rover teleop and SLAM workflows.

## Tools

| Module | What it does |
|---|---|
| `rover_tools.controller_teleop` | Keyboard teleop + controller USB I/O + live web viewer (+ optional `--slam`) |
| `rover_tools.record` | Records controller JSON telemetry to timestamped JSONL |
| `rover_tools.replay` | Replays a recording through the full SLAM stack without hardware |

## Setup

From `py_scripts/`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
# Teleop + viewer
python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080

# Teleop + SLAM
python -m rover_tools.controller_teleop --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam

# Record a session
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl

# Replay offline at 2x speed
python -m rover_tools.replay --file run1.jsonl --speed 2.0
```

Open `http://localhost:8080` for the web viewer.

Legacy launchers are kept at `rover_stack/py/*.py` for backward compatibility.
