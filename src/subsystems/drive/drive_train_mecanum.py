from typing import override

import rev
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from wpilib import DriverStation, Field2d, SmartDashboard
from wpilib.drive import MecanumDrive
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveKinematics, MecanumDriveWheelPositions, MecanumDriveWheelSpeeds

from src.navx.navx import Navx
from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_LEFT_LOCATION, FRONT_RIGHT_ID, FRONT_RIGHT_LOCATION, MAX_ANGULAR_SPEED, MAX_SPEED, REAR_LEFT_ID, REAR_LEFT_LOCATION, REAR_RIGHT_ID, REAR_RIGHT_LOCATION, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO
from src.coordinate_systems.coordinate_systems_2d import Velocities2d


class DriveTrainMecanum(Subsystem):
    def __init__(self, pose_estimator: MecanumDrivePoseEstimator3d, navx: Navx) -> None:
        super().__init__()

        self.left_front_drive = rev.SparkMax(FRONT_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = rev.SparkMax(FRONT_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = rev.SparkMax(REAR_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = rev.SparkMax(REAR_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)

        config = rev.SparkMaxConfig()

        # conversion_ratio = WHEEL_CIRCUMFERENCE / WHEEL_GEAR_RATIO
        # config.encoder.positionConversionFactor(conversion_ratio)
        # config.encoder.velocityConversionFactor(conversion_ratio / 60)

        self.config_drive_motor(self.left_front_drive, False)
        self.config_drive_motor(self.right_front_drive, True)
        self.config_drive_motor(self.left_rear_drive, False)
        self.config_drive_motor(self.right_rear_drive, True)

        self.left_front_encoder = self.left_front_drive.getEncoder()
        self.right_front_encoder = self.right_front_drive.getEncoder()
        self.left_rear_encoder = self.left_rear_drive.getEncoder()
        self.right_rear_encoder = self.right_rear_drive.getEncoder()

        self.robot_drive = MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

        self.pose_estimator = pose_estimator
        self.navx = navx

        self.kinematics = MecanumDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            REAR_LEFT_LOCATION,
            REAR_RIGHT_LOCATION,
        )

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.get_pose_2d,
            self.reset_pose_2d,
            self.get_relative_speeds,
            lambda speeds, feedforwards: self.drive(Velocities2d.from_chassis_speeds(speeds)),
            PPHolonomicDriveController(PIDConstants(0.25, 0.0, 0.03), PIDConstants(0.25, 0.0, 0.01)),
            config,
            self.should_flip_path,
            self,
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def should_flip_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def drive(self, velocities: Velocities2d) -> None:
        self.robot_drive.driveCartesian(*velocities.linear('linear_speed_percent', LinearCoordinateConvention2d.X_FORWARD_Y_LEFT), velocities.angular('angular_speed_percent', AngularCoordianteConvention2d.CCW_POSITIVE))

    def drive_field_oriented(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed, self.navx.get_2d_rotation())

    @override
    def periodic(self) -> None:
        if self.pose_estimator is not None:
            self.pose_estimator.update(
                self.navx.get_full_rotation(),
                self.get_wheel_positions(),
            )
            self.field.setRobotPose(self.pose_estimator.getEstimatedPosition().toPose2d())

        # if RobotState.isEnabled():
        #     pose = self.pose_estimator.getEstimatedPosition()

        #     print("New thingy, printing pose then wheel positions")
        #     print(pose.x, pose.y, pose.z, self.navx.get_heading())

        #     positions = self.get_wheel_positions()
        #     print(positions.frontLeft, positions.frontRight, positions.rearLeft, positions.rearRight)

        #     speeds = self.get_relative_speeds()
        #     print("speeds:")
        #     print(
        #         self.left_front_encoder.getVelocity(),
        #         self.right_front_encoder.getVelocity(),
        #         self.left_rear_encoder.getVelocity(),
        #         self.right_rear_encoder.getVelocity(),
        #     )
        #     print(speeds.vx, speeds.vy, speeds.omega)

    def get_wheel_positions(self) -> MecanumDriveWheelPositions:
        positions = MecanumDriveWheelPositions()

        positions.frontLeft = self.left_front_encoder.getPosition()
        positions.frontRight = self.right_front_encoder.getPosition()
        positions.rearLeft = self.left_rear_encoder.getPosition()
        positions.rearRight = self.right_rear_encoder.getPosition()

        return positions

    def zero_encoder_positions(self) -> None:
        self.left_front_encoder.setPosition(0)
        self.right_front_encoder.setPosition(0)
        self.left_rear_encoder.setPosition(0)
        self.right_rear_encoder.setPosition(0)

    def get_wheel_speeds(self) -> MecanumDriveWheelSpeeds:
        return MecanumDriveWheelSpeeds(
            self.left_front_encoder.getVelocity(),
            self.right_front_encoder.getVelocity(),
            self.left_rear_encoder.getVelocity(),
            self.right_rear_encoder.getVelocity(),
        )

    def get_relative_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_wheel_speeds())

    def get_pose_2d(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition().toPose2d()

    def get_pose_3d(self) -> Pose3d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose_2d(self, pose: Pose2d) -> None:
        self.pose_estimator.resetPose(Pose3d(pose))

    def reset_pose_3d(self, pose: Pose3d) -> None:
        self.pose_estimator.resetPose(pose)

    def config_drive_motor(self, motor: rev.SparkMax, inverted: bool) -> None:
        config = rev.SparkMaxConfig()

        config.inverted(inverted)

        conversion_ratio = WHEEL_CIRCUMFERENCE / WHEEL_GEAR_RATIO
        config.encoder.positionConversionFactor(conversion_ratio)
        config.encoder.velocityConversionFactor(conversion_ratio / 60)

        motor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
