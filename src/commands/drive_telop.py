from typing import override

from commands2 import Command
from wpilib import Joystick, RobotBase

from src.constants import SPEED_SCALAR
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum


class DriveTelop(Command):
    def __init__(self, drive: DriveTrainMecanum, controller: Joystick) -> None:
        super().__init__()

        self.drive = drive
        self.controller = controller

        self.addRequirements(drive)

    @override
    def initialize(self) -> None:
        pass

    @override
    def execute(self) -> None:
        forward_speed = self.get_controller_axis(1)
        strafe_speed = -self.get_controller_axis(0)
        turn_speed = self.get_controller_axis(4)

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

    def get_controller_axis(self, axis):
        if self.controller.getAxisCount()==0 and RobotBase.isSimulation():
            print("no drive controller axes, using default")
            return 1 if axis==1 else 0
        else:
            return self.controller.getRawAxis(axis)

