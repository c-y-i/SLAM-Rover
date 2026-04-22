# rover_stack Python tools

Host-side tooling for the SLAM Rover workflow.

## Tools

| Script | What it does |
|---|---|
| `controller_teleop.py` | Keyboard teleop + controller USB I/O + live web viewer (+ optional `--slam`) |
| `record.py` | Records controller JSON telemetry to a timestamped JSONL file |
| `replay.py` | Replays a recording through the full SLAM stack without hardware |

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
# Teleop + viewer
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080

# Teleop + SLAM
python controller_teleop.py --port /dev/ttyACM0 --baud 460800 --web-port 8080 --slam

# Record a session
python record.py --port /dev/ttyACM0 --output run1.jsonl

# Replay offline at 2× speed
python replay.py --file run1.jsonl --speed 2.0
```

Open `http://localhost:8080` for the web viewer.
