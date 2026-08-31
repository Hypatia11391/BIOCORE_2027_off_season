from math import pi
from typing import Final

from wpimath.geometry import Translation2d

# tune this to cap max output for testing
MAX_SPEED: Final[float] = 1.0

# CAN IDs (spark max)
FRONT_LEFT_ID: Final[int] = 9
FRONT_RIGHT_ID: Final[int] = 1
REAR_LEFT_ID: Final[int] = 10
REAR_RIGHT_ID: Final[int] = 2

# In meters
WHEEL_DIAMETER: Final[float] = 0.15354  # 0.1588
WHEEL_RADIUS: Final[float] = WHEEL_DIAMETER / 2
WHEEL_CIRCUMFERENCE: Final[float] = pi * WHEEL_DIAMETER
WHEEL_GEAR_RATIO: Final[float] = 26 / 3  # TODO: Figure out correct gear ratio, might just be 1

# rads/sec
MAX_ANGULAR_SPEED: Final[float] = pi

DRIVE_MOTOR_CHANNEL_FRONT_LEFT: Final[int] = 0
TURNING_MOTOR_CHANNEL_FRONT_LEFT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_A_FRONT_LEFT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_B_FRONT_LEFT: Final[int] = 0
TURNING_ENCODER_CHANNEL_A_FRONT_LEFT: Final[int] = 0
TURNING_ENCODER_CHANNEL_B_FRONT_LEFT: Final[int] = 0

DRIVE_MOTOR_CHANNEL_FRONT_RIGHT: Final[int] = 0
TURNING_MOTOR_CHANNEL_FRONT_RIGHT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_A_FRONT_RIGHT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_B_FRONT_RIGHT: Final[int] = 0
TURNING_ENCODER_CHANNEL_A_FRONT_RIGHT: Final[int] = 0
TURNING_ENCODER_CHANNEL_B_FRONT_RIGHT: Final[int] = 0

DRIVE_MOTOR_CHANNEL_REAR_LEFT: Final[int] = 0
TURNING_MOTOR_CHANNEL_REAR_LEFT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_A_REAR_LEFT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_B_REAR_LEFT: Final[int] = 0
TURNING_ENCODER_CHANNEL_A_REAR_LEFT: Final[int] = 0
TURNING_ENCODER_CHANNEL_B_REAR_LEFT: Final[int] = 0

DRIVE_MOTOR_CHANNEL_REAR_RIGHT: Final[int] = 0
TURNING_MOTOR_CHANNEL_REAR_RIGHT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_A_REAR_RIGHT: Final[int] = 0
DRIVE_ENCODER_CHANNEL_B_REAR_RIGHT: Final[int] = 0
TURNING_ENCODER_CHANNEL_A_REAR_RIGHT: Final[int] = 0
TURNING_ENCODER_CHANNEL_B_REAR_RIGHT: Final[int] = 0

FRONT_LEFT_LOCATION: Final[Translation2d] = Translation2d(0.180, 0.340)
FRONT_RIGHT_LOCATION: Final[Translation2d] = Translation2d(0.180, -0.340)
REAR_LEFT_LOCATION: Final[Translation2d] = Translation2d(-0.180, 0.340)
REAR_RIGHT_LOCATION: Final[Translation2d] = Translation2d(-0.180, -0.340)

MAX_VELOCITY: Final[float] = 0
MAX_ANGULAR_ACCELERATION: Final[float] = 0
MAX_ANGULAR_VELOCITY: Final[float] = 0
