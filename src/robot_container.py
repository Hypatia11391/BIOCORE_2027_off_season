from wpilib import Joystick
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

import src.subsystems
from src.subsystems.pose_estimation.vision_server import VisionServer
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum
from src.subsystems.mechanisms.intake import Intake
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter
from src.navx.navx import Navx
from src.commands.drive_telop import DriveTelop
from src.commands.operate_telop import OperateTelop
import src.subsystems.drive.drive_train_constants as drive_constants
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

        self.vision_server = VisionServer(self.pose_estimator)

        self.drive = DriveTrainMecanum(self.pose_estimator, self.navx)
        self.intake = Intake()
        self.feed = Feed()
        self.kicker = Kicker()
        self.shooter = Shooter()

        self.controller_drive = Joystick(0)
        self.controller_operate = Joystick(1)

        self.drive.setDefaultCommand(DriveTelop(self.drive, self.controller_drive))
        self.shooter.setDefaultCommand(OperateTelop(self.intake, self.feed, self.kicker, self.shooter, self.controller_operate))
