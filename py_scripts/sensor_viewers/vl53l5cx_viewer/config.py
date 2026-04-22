"""Configuration constants for the VL53L5CX + MPU6050 viewer."""

from dataclasses import dataclass


VERSION = "0.2.0"

RESOLUTION = 8
NUM_ZONES = 64

MIN_RANGE_MM = 20
MAX_RANGE_MM = 4000
VALID_STATUS_CODES = (5, 9)

TARGET_FPS = 30
FRAME_TIME = 1.0 / TARGET_FPS
SERIAL_TIMEOUT_S = 1.0
TOF_TIMEOUT_S = 1.5
IMU_TIMEOUT_S = 1.0
MADGWICK_BETA = 0.08
DEFAULT_IMU_DT_S = 0.01


@dataclass(frozen=True)
class BoardConfig:
    """Simple physical model for a sensor board."""

    world_position: tuple[float, float, float]
    sensor_offset: tuple[float, float, float]
    dimensions: tuple[float, float, float]
    color: tuple[int, int, int]
    sensor_yaw_deg: float = 0.0


IMU_BOARD = BoardConfig(
    world_position=(0.0, 0.0, 0.0),
    sensor_offset=(0.0, 0.0, 0.0),
    dimensions=(0.018, 0.025, 0.004),
    color=(70, 135, 255),
    sensor_yaw_deg=0.0,
)

TOF_BOARD = BoardConfig(
    world_position=(0.0, -0.0254, 0.0),
    sensor_offset=(0.0, 0.0, 0.0),
    dimensions=(0.018, 0.018, 0.004),
    color=(70, 220, 150),
    sensor_yaw_deg=90.0,
)

# Default correction for aligning the MPU6050 frame to the viewer/world frame.
# Keep this configurable so it can be tuned after live testing.
IMU_QUAT_CORRECTION_EULER_DEG = (0.0, 0.0, 0.0)

# ST-calibrated VL53L5CX lookup tables.
ST_PITCH_ANGLES_DEG = [
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
    64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
    67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
    70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
    70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
    67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
    64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
]

ST_YAW_ANGLES_DEG = [
    135.00, 125.40, 113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
    144.60, 135.00, 120.96, 101.31, 78.69, 59.04, 45.00, 35.40,
    156.80, 149.04, 135.00, 108.45, 71.55, 45.00, 30.96, 23.20,
    171.87, 168.69, 161.55, 135.00, 45.00, 18.45, 11.31, 8.13,
    188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
    203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
    215.40, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
    225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00,
]
