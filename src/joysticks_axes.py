from enum import Enum


class JoystickAxes(Enum):
    LEFT_X = 0
    LEFT_Y = LEFT_X + 1
    LT = LEFT_Y + 1
    RT = LT + 1
    RIGHT_X = RT + 1
    RIGHT_Y = RIGHT_X + 1
