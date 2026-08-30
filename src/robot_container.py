from commands2 import Command
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.logging import PathPlannerLogging
from wpilib import DriverStation, Field2d, Joystick
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Rotation2d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions

import src.constants as consts
import src.subsystems.drive.drive_train_constants as drive_consts
from src.commands.drive_telop import DriveTelop
from src.commands.operate_telop import OperateTelop
from src.navx.navx import Navx
from src.network_server.network_server import NetworkServer
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.intake import Intake
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter


class RobotContainer:
    def __init__(self):
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

        NetworkServer.getInstance().set_string_list("auto-list", ["Drive Back and Forth 5m"])

        self.field = Field2d()

        self.autonomous_command = Command()

        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject("trajectory").setPoses(poses))

    def get_autonomous_command(self) -> Command:
        self.autonomous_command = PathPlannerAuto(NetworkServer.getInstance().get_string("selected-auto"))
        return self.autonomous_command

    def zero_pose(self) -> None:
        self.pose_estimator.resetPose(consts.STARTING_POSE)
        self.navx.zero_yaw()
        self.drive.zero_encoder_positions()

    def periodic(self) -> None:
        self.field.setRobotPose(self.pose_estimator.getEstimatedPosition().toPose2d())
        self.field.getObject("velocity").setPose(
            self.drive.get_relative_speeds().vx,
            self.drive.get_relative_speeds().vy,
            Rotation2d(self.drive.get_relative_speeds().omega),
        )

        if not DriverStation.isAutonomous():
            self.field.getObject("trajectory").setPoses([])
