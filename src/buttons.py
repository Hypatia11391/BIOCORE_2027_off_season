from enum import Enum


class Buttons(Enum):
    A = 1
    B = A + 1
    X = B + 1
    Y = X + 1
    LB = Y + 1
    RB = LB + 1
    BACK = RB + 1
    START = BACK + 1
    LS = START + 1
    RS = LS + 1
