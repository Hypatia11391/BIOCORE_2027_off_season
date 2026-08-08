from math import radians

from commands2 import Subsystem
from wpimath.geometry import Rotation3d

from navx import AHRS


class Navx(Subsystem):
    def __init__(self) -> None:
        self.navx = AHRS(AHRS.NavXComType.kMXP_SPI)

    def get_roll_deg(self) -> float:
        return self.navx.getRoll()

    def get_pitch_deg(self) -> float:
        return self.navx.getPitch()

    def get_heading(self) -> float:
        return self.navx.getFusedHeading()

    def get_full_rotation(self) -> Rotation3d:
        return Rotation3d(
            radians(self.get_roll_deg()),
            radians(self.get_pitch_deg()),
            self.get_heading(),
        )

    def is_calibrating(self) -> bool:
        return self.navx.isCalibrating()

    def is_connected(self) -> bool:
        return self.navx.isConnected()

    def reset(self) -> None:
        self.navx.reset()

    def zero_yaw(self) -> None:
        self.navx.zeroYaw()

    def get_angle_deg(self) -> float:
        return self.navx.getAngle()

    def get_rate_deg(self) -> float:
        return self.navx.getRate()
