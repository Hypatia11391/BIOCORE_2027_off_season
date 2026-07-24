from commands2 import Command
from pathplannerlib.auto import PathPlannerAuto
from wpilib import Joystick
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

import src.constants as consts
import src.subsystems.drive.drive_train_constants as drive_consts
from src.commands.drive_telop import DriveTelop
from src.commands.operate_telop import OperateTelop
from src.navx.navx import Navx
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.intake import Intake
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter


class RobotContainer:
    def __init__(self) -> None:
        self.kinematics = MecanumDriveKinematics(
            drive_consts.FRONT_LEFT_LOCATION,
            drive_consts.FRONT_RIGHT_LOCATION,
            drive_consts.REAR_LEFT_LOCATION,
            drive_consts.REAR_RIGHT_LOCATION,
        )

        self.navx = Navx()

        self.pose_estimator = MecanumDrivePoseEstimator3d(
            self.kinematics,
            self.navx.get_full_rotation(),
            MecanumDriveWheelPositions(),
            consts.STARTING_POSE,
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

        # SmartDashboard.putStringArray("Auto List", ["Drive Forward 5m.auto"])

    def get_autonomous_command(self) -> Command:
        return PathPlannerAuto("Drive Forward 5m")
        # return PathPlannerAuto(SmartDashboard.getString("Auto Selector", "Drive Forward 5m.auto"))
