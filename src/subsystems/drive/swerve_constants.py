from typing import Final

from wpimath.geometry import Translation2d
from wpimath.units import meters
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

FL_DRIVE_ID: Final[int] = 1
FL_TURN_ID: Final[int] = 1
FL_CANCODER_ID: Final[int] = 1

FR_DRIVE_ID: Final[int] = 1
FR_TURN_ID: Final[int] = 1
FR_CANCODER_ID: Final[int] = 1

RL_DRIVE_ID: Final[int] = 1
RL_TURN_ID: Final[int] = 1
RL_CANCODER_ID: Final[int] = 1

RR_DRIVE_ID: Final[int] = 1
RR_TURN_ID: Final[int] = 1
RR_CANCODER_ID: Final[int] = 1

DRIVE_GEAR_RATIO: Final[float] = 1  # TODO: Fill in constants
TURN_GEAR_RATIO: Final[float] = 1
WHEEL_RADIUS: Final[meters] = 1

FL_LOCATION: Translation2d = Translation2d(0.18415, 0.3175)
FR_LOCATION: Translation2d = Translation2d(0.18415, -0.3175)
RL_LOCATION: Translation2d = Translation2d(-0.18415, 0.3175)
RR_LOCATION: Translation2d = Translation2d(-0.18415, -0.3175)

DRIVE_K_P: Final[float] = 1
DRIVE_K_I: Final[float] = 1
DRIVE_K_D: Final[float] = 1

DRIVE_K_S: Final[float] = 1
DRIVE_K_V: Final[float] = 1
DRIVE_K_A: Final[float] = 1

TURN_K_P: Final[float] = 1
TURN_K_I: Final[float] = 1
TURN_K_D: Final[float] = 1

TURN_K_S: Final[float] = 1
TURN_K_V: Final[float] = 1
TURN_K_A: Final[float] = 1

PATH_PLANNER_DRIVE_CONTROLLER: Final[PPHolonomicDriveController] = PPHolonomicDriveController(
    PIDConstants(0.0, 0.0, 0.0),
    PIDConstants(0.0, 0.0, 0.0),
)
