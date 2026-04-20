#!/usr/bin/env python3
"""Main 3D viewer application for the VL53L5CX + MPU6050 rig."""

from __future__ import annotations

import argparse
import logging
import time

import numpy as np
import viser

from . import config
from .geometry import apply_imu_correction, compute_zone_geometry, distances_to_points, get_colors, valid_mask
from .scene import create_scene, set_rays_visible
from .serial_reader import SerialReader

logger = logging.getLogger("vl53l5cx_viewer")


class VL53L5CXViewer:
    def __init__(self, port: str, baud: int = 460800) -> None:
        self.serial_reader = SerialReader(port, baud)
        self.zone_geometry = compute_zone_geometry()
        self.scene = None

    def _setup_scene(self, server: viser.ViserServer) -> None:
        self.scene = create_scene(server, self.zone_geometry)

    def _setup_gui(self, server: viser.ViserServer) -> None:
        with server.gui.add_folder("Sensor Info"):
            self.status_text = server.gui.add_text("Last Event", initial_value="Waiting for data")
            self.bus_status_text = server.gui.add_text("I2C Scan", initial_value="Waiting for scan")
            self.tof_text = server.gui.add_text("ToF", initial_value="Disconnected")
            self.tof_status_text = server.gui.add_text("ToF Status", initial_value="Waiting for data")
            self.imu_text = server.gui.add_text("IMU", initial_value="Disconnected")
            self.imu_status_text = server.gui.add_text("IMU Status", initial_value="Waiting for data")
            self.range_text = server.gui.add_text("Range", initial_value="--")
            self.tof_fps_text = server.gui.add_text("ToF FPS", initial_value="0.0")
            self.imu_fps_text = server.gui.add_text("IMU FPS", initial_value="0.0")

        with server.gui.add_folder("View"):
            self.point_size_slider = server.gui.add_slider(
                "Point Size",
                min=0.001,
                max=0.02,
                step=0.001,
                initial_value=0.005,
            )
            self.show_rays_checkbox = server.gui.add_checkbox(
                "Show Zone Rays",
                initial_value=True,
            )
            self.apply_imu_checkbox = server.gui.add_checkbox(
                "Apply IMU Rotation",
                initial_value=True,
            )

    def _process_frame(self, server: viser.ViserServer) -> None:
        snapshot = self.serial_reader.get_snapshot()

        self.status_text.value = snapshot.status_text
        self.bus_status_text.value = snapshot.bus_status_text
        self.tof_text.value = "Connected" if snapshot.tof_connected else "Disconnected"
        self.tof_status_text.value = snapshot.tof_status_text
        self.imu_text.value = "Connected" if snapshot.imu_connected else "Disconnected"
        self.imu_status_text.value = snapshot.imu_status_text
        self.tof_fps_text.value = f"{snapshot.tof_fps:.1f}"
        self.imu_fps_text.value = f"{snapshot.imu_fps:.1f}"
        set_rays_visible(self.scene.zone_rays, self.show_rays_checkbox.value)

        if self.apply_imu_checkbox.value and snapshot.imu_connected:
            corrected_quat = apply_imu_correction(snapshot.quaternion)
            self.scene.rig.wxyz = tuple(corrected_quat)
        else:
            self.scene.rig.wxyz = (1.0, 0.0, 0.0, 0.0)

        if not snapshot.tof_connected:
            self.range_text.value = "--"
            server.scene.remove_by_name("/rig/tof_board/sensor/points")
            return

        points = distances_to_points(snapshot.distances, self.zone_geometry)
        colors = get_colors(snapshot.distances, snapshot.status)
        mask = valid_mask(snapshot.distances, snapshot.status)

        if np.any(mask):
            valid_points = points[mask].astype(np.float32)
            valid_colors = colors[mask]
            valid_distances = snapshot.distances[mask]

            server.scene.add_point_cloud(
                "/rig/tof_board/sensor/points",
                points=valid_points,
                colors=valid_colors,
                point_size=float(self.point_size_slider.value),
                point_shape="circle",
            )
            self.range_text.value = (
                f"{float(valid_distances.min()):.0f}-{float(valid_distances.max()):.0f} mm"
            )
        else:
            self.range_text.value = "No valid points"
            server.scene.remove_by_name("/rig/tof_board/sensor/points")

    def run(self, host: str = "0.0.0.0", viser_port: int = 8080) -> None:
        self.serial_reader.connect()
        self.serial_reader.start()

        server = viser.ViserServer(host=host, port=viser_port)
        self._setup_scene(server)
        self._setup_gui(server)

        @server.on_client_connect
        def _on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.18, -0.32, 0.24)
            client.camera.look_at = (0.0, 0.0, 0.05)
            client.camera.up = (0.0, 0.0, 1.0)
            client.camera.near = 0.001
            client.camera.fov = 0.65

        logger.info("Viewer running at http://localhost:%d", viser_port)

        try:
            while True:
                frame_start = time.time()
                self._process_frame(server)

                elapsed = time.time() - frame_start
                if elapsed < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - elapsed)
        except KeyboardInterrupt:
            logger.info("Stopping viewer")
        finally:
            self.serial_reader.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="VL53L5CX + MPU6050 3D Viewer")
    parser.add_argument("--port", "-p", required=True, help="Serial port for the ESP32")
    parser.add_argument("--baud", "-b", type=int, default=460800, help="Serial baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="Viser server host")
    parser.add_argument("--viser-port", type=int, default=8080, help="Viser server port")
    parser.add_argument("--debug", action="store_true", help="Enable verbose logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    viewer = VL53L5CXViewer(port=args.port, baud=args.baud)
    viewer.run(host=args.host, viser_port=args.viser_port)
