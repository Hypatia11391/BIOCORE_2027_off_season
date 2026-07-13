import wpilib.drive as drive
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.kinematics import MecanumDriveWheelPositions
from commands2 import Subsystem
import rev

from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_RIGHT_ID, REAR_LEFT_ID, REAR_RIGHT_ID, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO
from src.navx.navx import Navx

from typing import override


class DriveTrainMecanum(Subsystem):
    def __init__(self, pose_estimator: MecanumDrivePoseEstimator3d, navx: Navx) -> None:
        super().__init__()

        self.left_front_drive = rev.SparkMax(FRONT_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = rev.SparkMax(FRONT_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = rev.SparkMax(REAR_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = rev.SparkMax(REAR_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)

        config = rev.SparkMaxConfig()

        conversion_ratio = WHEEL_CIRCUMFERENCE / WHEEL_GEAR_RATIO
        config.encoder.positionConversionFactor(conversion_ratio)
        config.encoder.velocityConversionFactor(conversion_ratio)

        self.left_front_drive.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.right_front_drive.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.left_rear_drive.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.right_rear_drive.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.left_front_encoder = self.left_front_drive.getEncoder()
        self.right_front_encoder = self.right_front_drive.getEncoder()
        self.left_rear_encoder = self.left_rear_drive.getEncoder()
        self.right_rear_encoder = self.right_rear_drive.getEncoder()

        self.robot_drive = drive.MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

        self.pose_estimator = pose_estimator
        self.navx = navx

    def drive(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed)

    @override
    def periodic(self) -> None:
        if self.pose_estimator is not None:
            self.pose_estimator.update(
                self.navx.get_full_rotation(),
                self.get_wheel_positions(),
            )

    def get_wheel_positions(self) -> MecanumDriveWheelPositions:
        positions = MecanumDriveWheelPositions()

        positions.frontLeft = self.left_front_encoder.getPosition()
        positions.frontRight = self.right_front_encoder.getPosition()
        positions.rearLeft = self.left_rear_encoder.getPosition()
        positions.rearRight = self.right_rear_encoder.getPosition()
        
        return positions
