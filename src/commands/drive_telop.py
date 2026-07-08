from commands2 import Command
from wpilib import Joystick

from typing import override

from src.subsytems.drive.drive_train_mecanum import DriveTrainMecanum
from src.constants import SPEED_SCALAR


class DriveTelop(Command):
    def __init__(self, drive: DriveTrainMecanum, controller: Joystick):
        super().__init__()

        self.drive = drive
        self.controller = controller

        self.addRequirements(drive)

    @override
    def initialize(self) -> None:
        pass

    @override
    def execute(self) -> None:
        forward_speed = -self.controller.getRawAxis(1)
        strafe_speed = self.controller.getRawAxis(0)
        turn_speed = -self.controller.getRawAxis(2)

        forward_speed = 0 if abs(forward_speed) < 0.05 else forward_speed
        strafe_speed = 0 if abs(strafe_speed) < 0.05 else strafe_speed
        turn_speed = 0 if abs(turn_speed) < 0.05 else turn_speed

        self.drive.drive(
            forward_speed * SPEED_SCALAR,
            strafe_speed * SPEED_SCALAR,
            turn_speed * SPEED_SCALAR,
        )

    @override
    def end(self, interrupted: bool) -> None:
        pass

    @override
    def isFinished(self) -> bool:
        return False
