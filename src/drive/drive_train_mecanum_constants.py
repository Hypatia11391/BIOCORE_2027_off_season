from typing import Final
from math import pi

# tune this to cap max output for testing
MAX_SPEED: Final[float] = 1.0

# CAN IDs (spark max)
FRONT_LEFT_ID: Final[int] = 9
FRONT_RIGHT_ID: Final[int] = 1
REAR_LEFT_ID: Final[int] = 10
REAR_RIGHT_ID: Final[int] = 2

# In meters
WHEEL_DIAMETER: Final[float] = 0.1588
WHEEL_CIRCUMFERENCE: Final[float] = pi * WHEEL_DIAMETER

# rads/sec
MAX_ANGULAR_SPEED: Final[float] = pi
