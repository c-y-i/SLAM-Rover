from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
import serial
import viser


MODE_KEYS = {"1", "2", "x"}
RAMP_KEYS = {"w", "a", "s", "d", "q", "e", "x", " "}
DIRECT_MOTOR_PREFIXES = {"L", "R"}

SCAN_CONNECTED_TIMEOUT_S = 2.0
KEY_STEP_PWM = 24
SPIN_STEP_PWM = 36
RAMP_STEP_PWM = 10
MAX_PWM = 220
SEND_INTERVAL_S = 0.08
PANEL_REFRESH_S = 0.1
LOOP_SLEEP_S = 0.02
PANEL_HEARTBEAT_S = 1.0


@dataclass
class TelemetryState:
    scan_count: int = 0
    last_scan_wall: float = 0.0
    last_imu_wall: float = 0.0
    last_status: str = "waiting_for_serial"
    imu_ok: bool = False
    yaw_deg: float = 0.0
    yaw_rate_dps: float = 0.0


@dataclass
class DriveState:
    target_left: int = 0
    target_right: int = 0
    current_left: int = 0
    current_right: int = 0
    last_sent_left: int | None = None
    last_sent_right: int | None = None
    last_send_wall: float = 0.0


@dataclass(frozen=True)
class ScanSnapshot:
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


class ControllerTeleopBridge:
    def __init__(self, port: str, baud: int = 460800) -> None:
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.running = False
        self._reader_thread: threading.Thread | None = None
        self._log_queue: queue.Queue[str] = queue.Queue()
        self._state_lock = threading.Lock()

        self.telemetry = TelemetryState()
        self.drive = DriveState()
        self.current_mode = "x"

        self._x_m = np.zeros(0, dtype=np.float32)
        self._y_m = np.zeros(0, dtype=np.float32)
        self._distance_m = np.zeros(0, dtype=np.float32)
        self._intensity = np.zeros(0, dtype=np.uint8)

    def connect(self) -> None:
        self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)
        self.serial.reset_input_buffer()

    def start(self) -> None:
        if self.serial is None:
            raise RuntimeError("Serial is not connected")
        if self._reader_thread is not None:
            return
        self.running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def stop(self) -> None:
        self.running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
            self.serial = None

    def send_line(self, payload: str) -> None:
        if self.serial is None:
            raise RuntimeError("Serial is not connected")
        self.serial.write(payload.encode("utf-8"))
        self.serial.flush()

    def send_mode(self, mode: str) -> None:
        self.current_mode = mode
        self.send_line(mode)
        self._log_queue.put(f"mode -> {mode}")
        if mode == "x":
            self.set_targets(0, 0)
            self.drive.current_left = 0
            self.drive.current_right = 0
            self._send_wheel_speeds(force=True)

    def set_targets(self, left: int, right: int) -> None:
        self.drive.target_left = max(-MAX_PWM, min(MAX_PWM, left))
        self.drive.target_right = max(-MAX_PWM, min(MAX_PWM, right))

    def nudge_targets(self, key: str) -> None:
        if key in {" ", "x"}:
            self.set_targets(0, 0)
            return

        if self.current_mode != "2":
            self.send_mode("2")

        left = self.drive.target_left
        right = self.drive.target_right

        if key == "w":
            left += KEY_STEP_PWM
            right += KEY_STEP_PWM
        elif key == "s":
            left -= KEY_STEP_PWM
            right -= KEY_STEP_PWM
        elif key == "a":
            left_delta, right_delta = self._steer_deltas("a")
            left += left_delta
            right += right_delta
        elif key == "d":
            left_delta, right_delta = self._steer_deltas("d")
            left += left_delta
            right += right_delta
        elif key == "q":
            left -= SPIN_STEP_PWM
            right += SPIN_STEP_PWM
        elif key == "e":
            left += SPIN_STEP_PWM
            right -= SPIN_STEP_PWM

        self.set_targets(left, right)
        self._log_queue.put(
            f"target -> L {self.drive.target_left:+d}  R {self.drive.target_right:+d}"
        )

    def _steer_deltas(self, key: str) -> tuple[int, int]:
        reversing = self._is_reversing()
        if key == "a":
            return (KEY_STEP_PWM, -KEY_STEP_PWM) if reversing else (-KEY_STEP_PWM, KEY_STEP_PWM)
        return (-KEY_STEP_PWM, KEY_STEP_PWM) if reversing else (KEY_STEP_PWM, -KEY_STEP_PWM)

    def send_direct_motor(self, command: str) -> None:
        self.send_line(command + "\n")
        self._log_queue.put(f"motor -> {command}")

    def tick(self) -> None:
        now = time.time()
        self.drive.current_left = _ramp_toward(
            self.drive.current_left, self.drive.target_left, RAMP_STEP_PWM
        )
        self.drive.current_right = _ramp_toward(
            self.drive.current_right, self.drive.target_right, RAMP_STEP_PWM
        )

        changed = (
            self.drive.current_left != self.drive.last_sent_left
            or self.drive.current_right != self.drive.last_sent_right
        )
        should_keepalive = (
            not self._is_fully_stopped()
            and (now - self.drive.last_send_wall) >= SEND_INTERVAL_S
        )
        if changed or should_keepalive:
            self._send_wheel_speeds(force=changed)

    def _send_wheel_speeds(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self.drive.last_send_wall) < SEND_INTERVAL_S:
            return

        left_cmd = _format_motor_command("L", self.drive.current_left)
        right_cmd = _format_motor_command("R", self.drive.current_right)
        self.send_line(left_cmd + "\n")
        self.send_line(right_cmd + "\n")
        self.drive.last_sent_left = self.drive.current_left
        self.drive.last_sent_right = self.drive.current_right
        self.drive.last_send_wall = now

    def _is_fully_stopped(self) -> bool:
        return (
            self.drive.target_left == 0
            and self.drive.target_right == 0
            and self.drive.current_left == 0
            and self.drive.current_right == 0
        )

    def _is_reversing(self) -> bool:
        target_avg = self.drive.target_left + self.drive.target_right
        current_avg = self.drive.current_left + self.drive.current_right
        return target_avg < 0 or (target_avg == 0 and current_avg < 0)

    def get_snapshot(self) -> ScanSnapshot:
        with self._state_lock:
            connected = (time.time() - self.telemetry.last_scan_wall) < SCAN_CONNECTED_TIMEOUT_S
            imu_connected = (time.time() - self.telemetry.last_imu_wall) < SCAN_CONNECTED_TIMEOUT_S
            return ScanSnapshot(
                x_m=self._x_m.copy(),
                y_m=self._y_m.copy(),
                distance_m=self._distance_m.copy(),
                intensity=self._intensity.copy(),
                connected=connected,
                imu_connected=imu_connected,
                imu_ok=self.telemetry.imu_ok,
                yaw_deg=self.telemetry.yaw_deg,
                yaw_rate_dps=self.telemetry.yaw_rate_dps,
                status_text=self.telemetry.last_status,
                scan_count=self.telemetry.scan_count,
            )

    def drain_logs(self) -> list[str]:
        messages: list[str] = []
        while True:
            try:
                messages.append(self._log_queue.get_nowait())
            except queue.Empty:
                return messages

    def _handle_scan(self, data: dict) -> None:
        x_raw = data.get("x")
        y_raw = data.get("y")
        i_raw = data.get("i")
        if not (isinstance(x_raw, list) and isinstance(y_raw, list) and isinstance(i_raw, list)):
            return
        if not (len(x_raw) == len(y_raw) == len(i_raw)) or len(x_raw) == 0:
            return

        x_m = np.asarray(x_raw, dtype=np.float32) / 1000.0
        y_m = np.asarray(y_raw, dtype=np.float32) / 1000.0
        distance_m = np.hypot(x_m, y_m)
        intensity = np.asarray(i_raw, dtype=np.uint8)

        with self._state_lock:
            self._x_m = x_m
            self._y_m = y_m
            self._distance_m = distance_m
            self._intensity = intensity
            self.telemetry.scan_count += 1
            self.telemetry.last_scan_wall = time.time()

    def _handle_status(self, data: dict) -> None:
        stage = str(data.get("stage", ""))
        detail = str(data.get("detail", ""))
        status_text = f"{stage}: {detail}".strip(": ")
        with self._state_lock:
            self.telemetry.last_status = status_text
        self._log_queue.put(f"status -> {status_text}")

    def _handle_imu(self, data: dict) -> None:
        yaw_deg = float(data.get("yaw_deg", 0.0))
        yaw_rate_dps = float(data.get("yaw_rate_dps", 0.0))
        imu_ok = bool(data.get("imu_ok", False))
        with self._state_lock:
            self.telemetry.last_imu_wall = time.time()
            self.telemetry.imu_ok = imu_ok
            self.telemetry.yaw_deg = yaw_deg
            self.telemetry.yaw_rate_dps = yaw_rate_dps

    def _read_loop(self) -> None:
        assert self.serial is not None
        while self.running:
            try:
                raw = self.serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if not line.startswith("{"):
                    self._log_queue.put(line)
                    continue
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(packet, dict):
                    continue

                packet_type = packet.get("t")
                if packet_type == "scan":
                    self._handle_scan(packet)
                elif packet_type == "status":
                    self._handle_status(packet)
                elif packet_type == "imu":
                    self._handle_imu(packet)
            except (serial.SerialException, OSError) as exc:
                self._log_queue.put(f"serial error: {exc}")
                self.running = False


class TeleopWebViewer:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self.server = viser.ViserServer(host=host, port=port)
        self.robot_frame = self.server.scene.add_frame(
            "/robot",
            show_axes=True,
            axes_length=0.28,
            axes_radius=0.007,
        )
        self.server.scene.add_frame("/origin", axes_length=0.12, axes_radius=0.003)
        self._add_grid_rings()
        self._add_gui()

        @self.server.on_client_connect
        def _on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, 0.0, 7.0)
            client.camera.look_at = (0.0, 0.0, 0.0)
            client.camera.up = (0.0, 1.0, 0.0)
            client.camera.fov = 0.9

    @property
    def url(self) -> str:
        host = "localhost" if self._host in {"0.0.0.0", "::"} else self._host
        return f"http://{host}:{self._port}"

    def _add_grid_rings(self) -> None:
        radii_m = [1.0, 2.0, 3.0, 4.0, 5.0]
        for radius_m in radii_m:
            angles = np.linspace(0.0, 2.0 * math.pi, 97)
            x = radius_m * np.cos(angles)
            y = radius_m * np.sin(angles)
            points = np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32)
            self.server.scene.add_spline_catmull_rom(
                f"/grid/ring_{radius_m:.0f}m",
                positions=points,
                line_width=1.2,
                color=(30, 80, 90),
                closed=True,
            )

    def _add_gui(self) -> None:
        with self.server.gui.add_folder("Rover Stack"):
            self.status_text = self.server.gui.add_text("Status", initial_value="Waiting…")
            self.mode_text = self.server.gui.add_text("Mode", initial_value="x")
            self.imu_text = self.server.gui.add_text("IMU", initial_value="No data")
            self.heading_text = self.server.gui.add_text("Heading (deg)", initial_value="0.0")
            self.turn_rate_text = self.server.gui.add_text("Turn Rate (dps)", initial_value="0.0")
            self.scan_count_text = self.server.gui.add_text("Scan Count", initial_value="0")
            self.points_text = self.server.gui.add_text("Points", initial_value="0")
            self.range_text = self.server.gui.add_text("Range", initial_value="--")

        with self.server.gui.add_folder("View"):
            self.color_mode = self.server.gui.add_dropdown(
                "Color Mode",
                options=["Radar", "Distance", "Intensity"],
                initial_value="Radar",
            )
            self.max_dist_slider = self.server.gui.add_slider(
                "Max Distance (m)",
                min=1.0,
                max=12.0,
                step=0.5,
                initial_value=12.0,
            )
            self.point_size_slider = self.server.gui.add_slider(
                "Point Size",
                min=0.005,
                max=0.15,
                step=0.005,
                initial_value=0.04,
            )

    def render(self, snapshot: ScanSnapshot, mode: str) -> None:
        if snapshot.connected:
            self.status_text.value = f"Connected ({snapshot.status_text})"
        else:
            self.status_text.value = f"Disconnected ({snapshot.status_text})"
        self.mode_text.value = mode
        self.scan_count_text.value = str(snapshot.scan_count)
        self.points_text.value = str(snapshot.x_m.size)
        if snapshot.imu_connected:
            self.imu_text.value = "Healthy" if snapshot.imu_ok else "Connected (degraded)"
        else:
            self.imu_text.value = "No data"
        self.heading_text.value = f"{snapshot.yaw_deg:.1f}"
        self.turn_rate_text.value = f"{snapshot.yaw_rate_dps:.1f}"

        if not snapshot.connected or snapshot.x_m.size == 0:
            self.range_text.value = "--"
            self.server.scene.remove_by_name("/robot/scan/points")
            return

        mask = snapshot.distance_m <= float(self.max_dist_slider.value)
        if not np.any(mask):
            self.range_text.value = "No points in range"
            self.server.scene.remove_by_name("/robot/scan/points")
            return

        x = snapshot.x_m[mask]
        y = snapshot.y_m[mask]
        dist = snapshot.distance_m[mask]
        intensity = snapshot.intensity[mask]

        points = np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32)
        colors = _get_colors(dist, intensity, str(self.color_mode.value))
        self.server.scene.add_point_cloud(
            "/robot/scan/points",
            points=points,
            colors=colors,
            point_size=float(self.point_size_slider.value),
            point_shape="circle",
        )
        self.range_text.value = f"{float(dist.min()):.2f}–{float(dist.max()):.2f} m"


def _colors_radar(intensity: np.ndarray) -> np.ndarray:
    t = intensity.astype(np.float32) / 255.0
    r = np.zeros(len(t), dtype=np.uint8)
    g = (t * 190 + 30).astype(np.uint8)
    b = (t * 195 + 60).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def _colors_distance(distance_m: np.ndarray) -> np.ndarray:
    min_dist_m = 0.02
    max_dist_m = 12.0
    t = np.clip((distance_m - min_dist_m) / (max_dist_m - min_dist_m), 0.0, 1.0).astype(
        np.float32
    )
    r = (t * 255).astype(np.uint8)
    g = np.zeros(len(t), dtype=np.uint8)
    b = ((1.0 - t) * 255).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def _colors_intensity(intensity: np.ndarray) -> np.ndarray:
    return intensity[:, np.newaxis].repeat(3, axis=1)


def _get_colors(distance_m: np.ndarray, intensity: np.ndarray, mode: str) -> np.ndarray:
    if mode == "Intensity":
        return _colors_intensity(intensity)
    if mode == "Distance":
        return _colors_distance(distance_m)
    return _colors_radar(intensity)


def _ramp_toward(current: int, target: int, step: int) -> int:
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


def _format_motor_command(motor: str, value: int) -> str:
    if value > 0:
        return f"{motor}f{value}"
    if value < 0:
        return f"{motor}b{abs(value)}"
    return f"{motor}s"


def _read_key_windows() -> str | None:
    import msvcrt

    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getwch()
    if ch in ("\x00", "\xe0"):
        _ = msvcrt.getwch()
        return None
    return ch


class _PosixKeyboard:
    def __init__(self) -> None:
        import termios
        import tty

        self._termios = termios
        self._tty = tty
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)

    def __enter__(self) -> "_PosixKeyboard":
        self._tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_settings)

    @staticmethod
    def read_key() -> str | None:
        import select

        readable, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not readable:
            return None
        return sys.stdin.read(1)


def _direction_label(left: int, right: int) -> str:
    if left == 0 and right == 0:
        return "stopped"
    if left > 0 and right > 0:
        if abs(left - right) <= 12:
            return "forward"
        return "arc_left" if right > left else "arc_right"
    if left < 0 and right < 0:
        if abs(left - right) <= 12:
            return "reverse"
        return "reverse_left" if right < left else "reverse_right"
    if left < 0 < right:
        return "spin_left"
    if right < 0 < left:
        return "spin_right"
    if left == 0:
        return "pivot_left" if right > 0 else "pivot_right"
    if right == 0:
        return "pivot_right" if left > 0 else "pivot_left"
    return "mixed"


def _render_panel(
    teleop: ControllerTeleopBridge,
    snapshot: ScanSnapshot,
    log_lines: list[str],
    viewer_url: str,
) -> str:
    lidar_state = "connected" if snapshot.connected else "not_connected"
    lines = [
        "rover_stack Teleop + Web Viewer",
        "",
        f"port       : {teleop.port} @ {teleop.baud}",
        f"web viewer : {viewer_url}",
        f"mode       : {teleop.current_mode}",
        f"direction  : {_direction_label(teleop.drive.current_left, teleop.drive.current_right)}",
        f"lidar      : {lidar_state}",
        f"imu        : {'ok' if snapshot.imu_ok and snapshot.imu_connected else ('connected' if snapshot.imu_connected else 'no_data')}",
        f"heading    : {snapshot.yaw_deg:+.1f} deg",
        f"turn_rate  : {snapshot.yaw_rate_dps:+.1f} dps",
        f"scan_count : {snapshot.scan_count}",
        f"status     : {snapshot.status_text}",
        "",
        "Wheel State",
        f"  left   current={teleop.drive.current_left:+4d}  target={teleop.drive.target_left:+4d}",
        f"  right  current={teleop.drive.current_right:+4d}  target={teleop.drive.target_right:+4d}",
        "",
        "Controls",
        "  1 lidar wander   2 ramped teleop   x/space stop",
        "  w/s forward/reverse ramp",
        "  a/d steer left/right",
        "  q/e spin left/right",
        "  m direct motor command",
        "  p print one-line summary",
        "  esc quit",
        "",
        "Recent Serial",
    ]
    recent = log_lines[-8:] if log_lines else ["(no recent serial lines)"]
    lines.extend(f"  {line}" for line in recent)
    return "\x1b[2J\x1b[H" + "\n".join(lines)


def _panel_signature(
    teleop: ControllerTeleopBridge,
    snapshot: ScanSnapshot,
    log_lines: list[str],
    viewer_url: str,
) -> tuple[object, ...]:
    recent = tuple(log_lines[-8:]) if log_lines else ("(no recent serial lines)",)
    return (
        teleop.port,
        teleop.baud,
        viewer_url,
        teleop.current_mode,
        _direction_label(teleop.drive.current_left, teleop.drive.current_right),
        snapshot.connected,
        snapshot.imu_connected,
        snapshot.imu_ok,
        round(snapshot.yaw_deg, 1),
        round(snapshot.yaw_rate_dps, 1),
        teleop.drive.current_left,
        teleop.drive.target_left,
        teleop.drive.current_right,
        teleop.drive.target_right,
        snapshot.status_text,
        recent,
    )


def _print_summary(teleop: ControllerTeleopBridge, snapshot: ScanSnapshot) -> None:
    print(
        f"[summary] mode={teleop.current_mode} "
        f"dir={_direction_label(teleop.drive.current_left, teleop.drive.current_right)} "
        f"L={teleop.drive.current_left:+d}/{teleop.drive.target_left:+d} "
        f"R={teleop.drive.current_right:+d}/{teleop.drive.target_right:+d} "
        f"scan_count={snapshot.scan_count} "
        f"lidar={'connected' if snapshot.connected else 'not_connected'} "
        f"imu={'ok' if snapshot.imu_ok and snapshot.imu_connected else 'not_connected'} "
        f"heading={snapshot.yaw_deg:+.1f}deg "
        f"turn={snapshot.yaw_rate_dps:+.1f}dps "
        f"status={snapshot.status_text}"
    )


def _prompt_direct_command() -> str:
    print("\nDirect motor command (example: Lf200, Rs): ", end="", flush=True)
    return input().strip()


def _run_loop(teleop: ControllerTeleopBridge, viewer: TeleopWebViewer) -> None:
    print("Connecting rover_stack teleop bridge...")
    print(f"Viewer available at {viewer.url}")

    log_lines: list[str] = []
    last_panel_wall = 0.0
    last_panel_heartbeat_wall = 0.0
    last_panel_signature: tuple[object, ...] | None = None
    is_windows = os.name == "nt"
    posix_keyboard = None

    if not is_windows:
        posix_keyboard = _PosixKeyboard()
        posix_keyboard.__enter__()

    try:
        while teleop.running:
            key = _read_key_windows() if is_windows else _PosixKeyboard.read_key()
            if key is not None:
                key = key.lower()
                if key == "\x1b":
                    break
                if key in MODE_KEYS:
                    teleop.send_mode(key)
                elif key in RAMP_KEYS:
                    teleop.nudge_targets(key)
                elif key == "m":
                    if posix_keyboard is not None:
                        posix_keyboard.__exit__(None, None, None)
                    command = _prompt_direct_command()
                    if command and command[0] in DIRECT_MOTOR_PREFIXES:
                        teleop.send_direct_motor(command)
                    if posix_keyboard is not None:
                        posix_keyboard.__enter__()
                elif key == "p":
                    _print_summary(teleop, teleop.get_snapshot())

            teleop.tick()
            snapshot = teleop.get_snapshot()
            viewer.render(snapshot, teleop.current_mode)

            for message in teleop.drain_logs():
                if not message.startswith("status -> ") or "controller: ready" not in message:
                    if not log_lines or log_lines[-1] != message:
                        log_lines.append(message)

            now = time.time()
            if now - last_panel_wall >= PANEL_REFRESH_S:
                last_panel_wall = now
                panel_signature = _panel_signature(teleop, snapshot, log_lines, viewer.url)
                heartbeat_due = (now - last_panel_heartbeat_wall) >= PANEL_HEARTBEAT_S
                if panel_signature != last_panel_signature or heartbeat_due:
                    panel = _render_panel(teleop, snapshot, log_lines, viewer.url)
                    print(panel, end="", flush=True)
                    last_panel_signature = panel_signature
                    last_panel_heartbeat_wall = now

            time.sleep(LOOP_SLEEP_S)
    finally:
        if posix_keyboard is not None:
            posix_keyboard.__exit__(None, None, None)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Keyboard teleop + web lidar viewer for rover_stack"
    )
    parser.add_argument("--port", "-p", required=True, help="Controller serial port")
    parser.add_argument("--baud", "-b", type=int, default=460800, help="Serial baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="Web viewer host")
    parser.add_argument("--web-port", type=int, default=8080, help="Web viewer port")
    parser.add_argument("--slam", action="store_true", help="Enable IMU-assisted ICP SLAM")
    args = parser.parse_args()

    teleop = ControllerTeleopBridge(port=args.port, baud=args.baud)
    viewer = TeleopWebViewer(host=args.host, port=args.web_port)

    slam = None
    if args.slam:
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
        from slam.slam_thread import SlamThread
        slam = SlamThread(teleop, viewer.robot_frame, viewer.server)

    try:
        teleop.connect()
        teleop.start()
        if slam is not None:
            slam.start()
        _run_loop(teleop, viewer)
    except KeyboardInterrupt:
        pass
    finally:
        if slam is not None:
            slam.stop()
        teleop.stop()


if __name__ == "__main__":
    main()
