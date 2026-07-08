from wpilib import Joystick
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

from subsytems.drive.drive_train_mecanum import DriveTrainMecanum
from src.navx.navx import Navx
from src.commands.drive_telop import DriveTelop
import subsytems.drive.drive_train_constants as drive_constants
import src.constants as constants


class RobotContainer:
    def __init__(self):
        self.kinematics = MecanumDriveKinematics(
            drive_constants.FRONT_LEFT_LOCATION,
            drive_constants.FRONT_RIGHT_LOCATION,
            drive_constants.REAR_LEFT_LOCATION,
            drive_constants.REAR_RIGHT_LOCATION,
        )

        self.navx = Navx()

        self.pose_estimator = MecanumDrivePoseEstimator3d(
            self.kinematics,
            self.navx.get_full_rotation(),
            MecanumDriveWheelPositions(),
            constants.STARTING_POSE,
        )

        self.drive = DriveTrainMecanum(self.pose_estimator, self.navx)

        self.controller = Joystick(0)

        self.drive.setDefaultCommand(DriveTelop(self.drive, self.controller))
