from wpilib import Joystick
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

from src.subsytems.drive.drive_train_mecanum import DriveTrainMecanum
from src.subsytems.mechanisms.intake import Intake
from src.subsytems.mechanisms.feed import Feed
from src.subsytems.mechanisms.kicker import Kicker
from src.subsytems.mechanisms.shooter import Shooter
from src.navx.navx import Navx
from src.commands.drive_telop import DriveTelop
from src.commands.operate_telop import OperateTelop
import subsytems.drive.drive_train_constants as drive_constants
import src.constants as constants


class RobotContainer:
    def __init__(self) -> None:
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
        self.intake = Intake()
        self.feed = Feed()
        self.kicker = Kicker()
        self.shooter = Shooter()

        self.controller_drive = Joystick(0)
        self.controller_operate = Joystick(1)

        self.drive.setDefaultCommand(DriveTelop(self.drive, self.controller_drive))
        self.shooter.setDefaultCommand(OperateTelop(self.intake, self.feed, self.kicker, self.shooter, self.controller_operate))
