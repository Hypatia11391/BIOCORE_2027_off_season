import wpilib
import wpilib.drive as drive
import rev

from src.drive.drive_train_mecanum import DriveTrainMecanum
from src.constants import speed_scaler


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")

        self.drive_train = DriveTrainMecanum()

        self.controller = wpilib.Joystick(0)

        self.i = 0

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        forward_speed = -self.controller.getRawAxis(1)
        strafe_speed = self.controller.getRawAxis(0)
        turn_speed = -self.controller.getRawAxis(2)

        if not self.i:
            print(f"{forward_speed=}\n{strafe_speed=}\n{turn_speed=}")
            self.i = 10

        else:
            self.i -= 1

        forward_speed = 0 if abs(forward_speed) < 0.05 else forward_speed
        strafe_speed = 0 if abs(strafe_speed) < 0.05 else strafe_speed
        turn_speed = 0 if abs(turn_speed) < 0.05 else turn_speed

        self.drive_train.drive(
            forward_speed * speed_scaler,
            strafe_speed * speed_scaler,
            turn_speed * speed_scaler,
        )

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def disabledPeriodic(self):
        pass

    def robotPeriodic(self):
        pass

    def simulationPeriodic(self):
        pass
