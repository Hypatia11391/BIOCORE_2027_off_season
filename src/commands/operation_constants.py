from enum import Enum
from typing import Final

INTAKE_LIFT_POS_UP: Final[float] = 126
INTAKE_LIFT_POS_DOWN: Final[float] = -72

INTAKE_FEED_PWR: Final[float] = 0.3


class IntakeFeedState(Enum):
    OFF = 0
    IN = OFF + 1
    OUT = IN + 1


class IntakeLiftState(Enum):
    OFF = 0
    UP = OFF + 1
    DOWN = UP + 1
