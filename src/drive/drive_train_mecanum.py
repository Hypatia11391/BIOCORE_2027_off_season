import wpilib
import wpilib.drive as drive
import rev


class DriveTrainMecanum:
    def __init__(self):
        self.left_front_drive = rev.SparkMax(9, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = rev.SparkMax(1, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)

        self.robot_drive = drive.MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

    def drive(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        self.robot_drive.driveCartesian(forward_speed, strafe_speed, turn_speed)
