"""Serial input pipeline for status, IMU, and ToF packets."""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass

import numpy as np
import serial

from . import config
from .imu_fusion import MadgwickFilter

logger = logging.getLogger("vl53l5cx_viewer.serial")


@dataclass(frozen=True)
class SensorSnapshot:
    distances: np.ndarray
    status: np.ndarray
    quaternion: np.ndarray
    tof_connected: bool
    imu_connected: bool
    tof_fps: float
    imu_fps: float
    status_text: str
    bus_status_text: str
    tof_status_text: str
    imu_status_text: str


class SerialReader:
    """Background thread that parses the mixed firmware JSON protocol."""

    def __init__(self, port: str, baud: int = 460800) -> None:
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        self.filter = MadgwickFilter(beta=config.MADGWICK_BETA)
        self._version_checked = False

        self._distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self._status = np.zeros(config.NUM_ZONES, dtype=np.uint8)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._status_text = "waiting_for_serial"
        self._bus_status_text = "waiting_for_scan"
        self._tof_status_text = "waiting_for_tof"
        self._imu_status_text = "waiting_for_imu"

        self._last_tof_wall = 0.0
        self._last_imu_wall = 0.0
        self._last_imu_ts_us: int | None = None

        self._tof_fps = 0.0
        self._imu_fps = 0.0
        self._tof_frame_count = 0
        self._imu_frame_count = 0
        self._tof_fps_window_start = time.time()
        self._imu_fps_window_start = time.time()

    def connect(self) -> None:
        logger.info("Connecting to %s at %d baud", self.port, self.baud)
        self.serial = serial.Serial(self.port, self.baud, timeout=config.SERIAL_TIMEOUT_S)
        time.sleep(2.0)
        self.serial.reset_input_buffer()
        logger.info("Serial connected")

    def start(self) -> None:
        if self._thread is not None:
            return
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.running = False
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_snapshot(self) -> SensorSnapshot:
        with self._lock:
            now = time.time()
            tof_connected = (now - self._last_tof_wall) < config.TOF_TIMEOUT_S
            imu_connected = (now - self._last_imu_wall) < config.IMU_TIMEOUT_S
            quaternion = self._quaternion.copy() if imu_connected else np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float32
            )
            return SensorSnapshot(
                distances=self._distances.copy(),
                status=self._status.copy(),
                quaternion=quaternion,
                tof_connected=tof_connected,
                imu_connected=imu_connected,
                tof_fps=self._tof_fps,
                imu_fps=self._imu_fps,
                status_text=self._status_text,
                bus_status_text=self._bus_status_text,
                tof_status_text=self._tof_status_text,
                imu_status_text=self._imu_status_text,
            )

    def _reset_runtime_state(self) -> None:
        with self._lock:
            self.filter.reset()
            self._quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            self._last_imu_ts_us = None
            self._last_tof_wall = 0.0
            self._last_imu_wall = 0.0
            self._tof_fps = 0.0
            self._imu_fps = 0.0
            self._tof_frame_count = 0
            self._imu_frame_count = 0
            self._status_text = "reconnecting"
            self._bus_status_text = "reconnecting"
            self._tof_status_text = "reconnecting"
            self._imu_status_text = "reconnecting"

    def _reconnect(self) -> bool:
        self._reset_runtime_state()
        try:
            if self.serial is not None:
                try:
                    self.serial.close()
                except serial.SerialException:
                    pass
            self.serial = serial.Serial(self.port, self.baud, timeout=config.SERIAL_TIMEOUT_S)
            time.sleep(2.0)
            self.serial.reset_input_buffer()
            logger.info("Serial reconnected")
            return True
        except (serial.SerialException, OSError) as exc:
            logger.debug("Reconnection failed: %s", exc)
            return False

    def _update_fps(self, packet_type: str) -> None:
        now = time.time()
        with self._lock:
            if packet_type == "tof":
                self._tof_frame_count += 1
                elapsed = now - self._tof_fps_window_start
                if elapsed >= 1.0:
                    self._tof_fps = self._tof_frame_count / elapsed
                    self._tof_frame_count = 0
                    self._tof_fps_window_start = now
            else:
                self._imu_frame_count += 1
                elapsed = now - self._imu_fps_window_start
                if elapsed >= 1.0:
                    self._imu_fps = self._imu_frame_count / elapsed
                    self._imu_frame_count = 0
                    self._imu_fps_window_start = now

    def _validate_numeric_list(self, values: list, expected_length: int) -> bool:
        if len(values) != expected_length:
            return False
        for value in values:
            if not isinstance(value, (int, float)):
                return False
            if math.isnan(value) or math.isinf(value):
                return False
        return True

    def _handle_status_packet(self, data: dict) -> None:
        stage = str(data.get("stage", "status"))
        detail = str(data.get("detail", ""))
        with self._lock:
            self._status_text = f"{stage}: {detail}".strip()
            if stage == "i2c_scan":
                self._bus_status_text = detail or "scan_unknown"
            elif stage == "tof":
                self._tof_status_text = detail or "status_unknown"
            elif stage == "imu":
                self._imu_status_text = detail or "status_unknown"

    def _handle_tof_packet(self, data: dict) -> None:
        distances = data.get("distances")
        status = data.get("status")
        if not isinstance(distances, list) or not isinstance(status, list):
            logger.debug("Skipping malformed ToF packet")
            return
        if not self._validate_numeric_list(distances, config.NUM_ZONES):
            logger.debug("Skipping invalid ToF distances packet")
            return
        if not self._validate_numeric_list(status, config.NUM_ZONES):
            logger.debug("Skipping invalid ToF status packet")
            return

        with self._lock:
            self._distances = np.asarray(distances, dtype=np.float32)
            self._status = np.asarray(status, dtype=np.uint8)
            self._last_tof_wall = time.time()
            self._tof_status_text = "streaming"

        self._update_fps("tof")

    def _handle_imu_packet(self, data: dict) -> None:
        accel = data.get("accel_mps2")
        gyro = data.get("gyro_rads")
        ts_us = data.get("ts_us")
        if not isinstance(accel, list) or not isinstance(gyro, list):
            logger.debug("Skipping malformed IMU packet")
            return
        if not self._validate_numeric_list(accel, 3):
            logger.debug("Skipping invalid IMU accel packet")
            return
        if not self._validate_numeric_list(gyro, 3):
            logger.debug("Skipping invalid IMU gyro packet")
            return
        if not isinstance(ts_us, int):
            logger.debug("Skipping IMU packet without integer timestamp")
            return

        accel_np = np.asarray(accel, dtype=np.float32)
        gyro_np = np.asarray(gyro, dtype=np.float32)

        with self._lock:
            if self._last_imu_ts_us is None or ts_us <= self._last_imu_ts_us:
                dt = config.DEFAULT_IMU_DT_S
            else:
                dt = (ts_us - self._last_imu_ts_us) / 1_000_000.0
                dt = min(max(dt, 1.0e-4), 0.1)

            quat = self.filter.update(gyro_np, accel_np, dt)
            self._quaternion = quat.astype(np.float32)
            self._last_imu_ts_us = ts_us
            self._last_imu_wall = time.time()
            self._imu_status_text = "streaming"

        self._update_fps("imu")

    def _handle_packet(self, data: dict) -> None:
        packet_type = data.get("t")
        firmware_version = data.get("v")

        if not self._version_checked and firmware_version is not None:
            self._version_checked = True
            if firmware_version != config.VERSION:
                logger.warning(
                    "Firmware/viewer version mismatch: firmware=%s viewer=%s",
                    firmware_version,
                    config.VERSION,
                )

        if packet_type == "status":
            self._handle_status_packet(data)
        elif packet_type == "tof":
            self._handle_tof_packet(data)
        elif packet_type == "imu":
            self._handle_imu_packet(data)

    def _read_loop(self) -> None:
        logger.info("Serial reader thread started")
        while self.running:
            try:
                if self.serial is None or not self.serial.is_open:
                    time.sleep(0.1)
                    continue

                raw_line = self.serial.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="ignore").strip()
                if not line.startswith("{"):
                    continue

                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    logger.debug("Skipping malformed JSON line")
                    continue

                if isinstance(packet, dict):
                    self._handle_packet(packet)
            except (serial.SerialException, OSError) as exc:
                if not self.running:
                    break
                logger.warning("Serial connection lost: %s", exc)
                while self.running:
                    if self._reconnect():
                        break
                    time.sleep(1.0)
