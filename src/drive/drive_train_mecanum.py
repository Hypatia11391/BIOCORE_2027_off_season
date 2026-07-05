import wpilib.drive as drive
import rev

from drive.drive_train_constants import FRONT_LEFT_ID, FRONT_RIGHT_ID, REAR_LEFT_ID, REAR_RIGHT_ID
from singleton_metaclass import SingletonMeta


class DriveTrainMecanum(metaclass=SingletonMeta):
    def __init__(self):
        self.left_front_drive = rev.SparkMax(FRONT_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = rev.SparkMax(FRONT_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = rev.SparkMax(REAR_LEFT_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = rev.SparkMax(REAR_RIGHT_ID, rev.SparkLowLevel.MotorType.kBrushless)

        self.robot_drive = drive.MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

    def drive(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed)
