#!/usr/bin/env python3
"""
Record rover_stack controller telemetry to a JSONL file for offline SLAM replay.

Each JSON line from the controller gets a "wall_s" timestamp prepended before
being written.  The file can then be fed into replay.py.

Usage:
    python -m rover_tools.record --port /dev/ttyACM0 [--baud 460800] [--output recording.jsonl]

Compatibility launcher:
    python rover_stack/py/record.py ...

If --output is omitted, the file is named recording_<unix_timestamp>.jsonl in
the current directory.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="Record rover_stack telemetry to JSONL")
    parser.add_argument("--port",   "-p", required=True,             help="Controller serial port")
    parser.add_argument("--baud",   "-b", type=int, default=460800)
    parser.add_argument("--output", "-o", default="",                help="Output file (default: recording_<ts>.jsonl)")
    args = parser.parse_args()

    out_path = Path(args.output) if args.output else Path(f"recording_{int(time.time())}.jsonl")
    print(f"Recording to  : {out_path}")
    print(f"Port          : {args.port} @ {args.baud} baud")
    print("Ctrl-C to stop\n")

    scan_count = imu_count = 0
    t_start = t_last_print = time.time()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()

    try:
        with out_path.open("w") as f:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line.startswith("{"):
                    continue
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(packet, dict):
                    continue

                packet["wall_s"] = time.time()
                f.write(json.dumps(packet) + "\n")
                f.flush()

                ptype = packet.get("t")
                if ptype == "scan":
                    scan_count += 1
                elif ptype == "imu":
                    imu_count += 1

                now = time.time()
                if now - t_last_print >= 1.0:
                    elapsed = now - t_start
                    print(
                        f"\r  {elapsed:.0f}s | scans: {scan_count} | imu: {imu_count}  ",
                        end="", flush=True,
                    )
                    t_last_print = now

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        print(f"\nStopped after {elapsed:.0f}s — {scan_count} scans, {imu_count} IMU frames → {out_path}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
