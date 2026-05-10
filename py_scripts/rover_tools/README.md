# Rover Tools

Host-side tooling for rover teleop and telemetry recording.

## Tools

| Module | What it does |
|---|---|
| `rover_tools.controller_teleop` | Keyboard teleop + controller USB I/O + live web viewer |
| `rover_tools.record` | Records controller JSON telemetry to timestamped JSONL |

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

# Record a session
python -m rover_tools.record --port /dev/ttyACM0 --output run1.jsonl
```

Open `http://localhost:8080` for the web viewer.
