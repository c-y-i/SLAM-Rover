#!/usr/bin/env python3
"""
Replay a recorded telemetry JSONL file through the SLAM stack.

Reads scan + IMU packets from a file produced by record.py, feeds them through
SlamThread, and displays the evolving map in the viser web viewer.

Usage:
    python replay.py --file recording.jsonl [--web-port 8080] [--speed 1.0]

    --speed 1.0   real-time
    --speed 2.0   2× faster
    --speed 0     as fast as possible (no sleep — good for quick map builds)
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import viser

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from slam.slam_thread import SlamThread  # noqa: E402


# ── snapshot type ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class ReplaySnapshot:
    """Duck-typed equivalent of controller_teleop.ScanSnapshot."""
    x_m: np.ndarray
    y_m: np.ndarray
    distance_m: np.ndarray
    intensity: np.ndarray
    connected: bool
    imu_connected: bool
    imu_ok: bool
    yaw_deg: float
    yaw_rate_dps: float
    status_text: str
    scan_count: int


_EMPTY = ReplaySnapshot(
    x_m=np.zeros(0, np.float32), y_m=np.zeros(0, np.float32),
    distance_m=np.zeros(0, np.float32), intensity=np.zeros(0, np.uint8),
    connected=False, imu_connected=False, imu_ok=False,
    yaw_deg=0.0, yaw_rate_dps=0.0, status_text="loading", scan_count=0,
)


# ── bridge ────────────────────────────────────────────────────────────────────

class ReplayBridge:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snap = _EMPTY

    def push(self, snap: ReplaySnapshot) -> None:
        with self._lock:
            self._snap = snap

    def get_snapshot(self) -> ReplaySnapshot:
        with self._lock:
            return self._snap


# ── replay runner ─────────────────────────────────────────────────────────────

def run_replay(path: Path, speed: float, server: viser.ViserServer) -> None:
    packets: list[dict] = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                packets.append(json.loads(line))
            except json.JSONDecodeError:
                continue

    if not packets:
        print("No packets in recording.")
        return

    n = len(packets)
    n_scans = sum(1 for p in packets if p.get("t") == "scan")
    print(f"Loaded {n} packets ({n_scans} scans) from {path.name}")

    bridge = ReplayBridge()
    robot_frame = server.scene.add_frame("/robot", axes_length=0.30, axes_radius=0.008, show_axes=True)
    server.scene.add_frame("/origin", axes_length=0.12, axes_radius=0.003)

    with server.gui.add_folder("Replay"):
        gui_file    = server.gui.add_text("File",         path.name)
        gui_progress = server.gui.add_text("Progress",    f"0 / {n_scans} scans")
        gui_elapsed  = server.gui.add_text("Elapsed",     "0s")
        gui_speed    = server.gui.add_text("Speed",       f"{speed:.1f}×" if speed > 0 else "max")
    with server.gui.add_folder("SLAM"):
        gui_pose_x  = server.gui.add_text("X (m)",       "--")
        gui_pose_y  = server.gui.add_text("Y (m)",       "--")
        gui_heading = server.gui.add_text("Heading (°)", "--")

    slam = SlamThread(bridge, robot_frame, server)
    slam.start()

    imu_yaw_deg   = 0.0
    imu_yaw_rate  = 0.0
    imu_ok        = False
    imu_seen      = False
    scan_count    = 0

    t_file_start   = packets[0].get("wall_s", 0.0)
    t_replay_start = time.time()

    try:
        for i, packet in enumerate(packets):
            ptype  = packet.get("t")
            wall_s = packet.get("wall_s", 0.0)

            # Pace to recorded speed (or scaled)
            if speed > 0 and i > 0:
                file_elapsed   = wall_s - t_file_start
                replay_elapsed = time.time() - t_replay_start
                delay = file_elapsed / speed - replay_elapsed
                if delay > 0:
                    time.sleep(delay)

            if ptype == "imu":
                imu_yaw_deg  = float(packet.get("yaw_deg",      0.0))
                imu_yaw_rate = float(packet.get("yaw_rate_dps", 0.0))
                imu_ok       = bool(packet.get("imu_ok",        False))
                imu_seen     = True

            elif ptype == "scan":
                x_raw = packet.get("x", [])
                y_raw = packet.get("y", [])
                i_raw = packet.get("i", [])
                if not (x_raw and y_raw and i_raw):
                    continue

                x_m    = np.asarray(x_raw, np.float32) / 1000.0
                y_m    = np.asarray(y_raw, np.float32) / 1000.0
                dist_m = np.hypot(x_m, y_m)
                inten  = np.asarray(i_raw, np.uint8)
                scan_count += 1

                bridge.push(ReplaySnapshot(
                    x_m=x_m, y_m=y_m, distance_m=dist_m, intensity=inten,
                    connected=True, imu_connected=imu_seen, imu_ok=imu_ok,
                    yaw_deg=imu_yaw_deg, yaw_rate_dps=imu_yaw_rate,
                    status_text="replay", scan_count=scan_count,
                ))

                sx, sy, st = slam.pose
                import math
                gui_progress.value = f"{scan_count} / {n_scans}"
                gui_elapsed.value  = f"{time.time() - t_replay_start:.0f}s"
                gui_pose_x.value   = f"{sx:.3f}"
                gui_pose_y.value   = f"{sy:.3f}"
                gui_heading.value  = f"{math.degrees(st):.1f}"

        print("Replay complete. Map frozen — Ctrl-C to exit.")
        gui_progress.value = f"done  ({n_scans} scans)"

        while True:
            time.sleep(1.0)

    finally:
        slam.stop()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded telemetry through SLAM")
    parser.add_argument("--file",     "-f", required=True, help="JSONL recording produced by record.py")
    parser.add_argument("--web-port",       type=int, default=8080)
    parser.add_argument("--speed",          type=float, default=1.0,
                        help="Replay speed multiplier; 0 = as fast as possible")
    args = parser.parse_args()

    path = Path(args.file)
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    server = viser.ViserServer(host="0.0.0.0", port=args.web_port)

    @server.on_client_connect
    def _on_connect(client: viser.ClientHandle) -> None:
        client.camera.position = (0.0, 0.0, 15.0)
        client.camera.look_at  = (0.0, 0.0, 0.0)
        client.camera.up       = (0.0, 1.0, 0.0)
        client.camera.fov      = 0.9

    speed_label = f"{args.speed:.1f}×" if args.speed > 0 else "max speed"
    print(f"Open http://localhost:{args.web_port}")
    print(f"Replaying {path.name} at {speed_label}")

    try:
        run_replay(path, args.speed, server)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
