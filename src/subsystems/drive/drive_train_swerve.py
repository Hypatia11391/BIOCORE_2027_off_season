import subsystems.drive.drive_train_constants as constants
from subsystems.drive.swerve_module import SwerveModule

from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from commands2 import Subsystem


class DriveTrainSwerve(Subsystem):
    def __init__(self) -> None:
        self.swerve_module_front_left = SwerveModule(
            constants.DRIVE_MOTOR_CHANNEL_FRONT_LEFT,
            constants.TURNING_MOTOR_CHANNEL_FRONT_LEFT,
            constants.DRIVE_ENCODER_CHANNEL_A_FRONT_LEFT,
            constants.DRIVE_ENCODER_CHANNEL_B_FRONT_LEFT,
            constants.TURNING_ENCODER_CHANNEL_A_FRONT_LEFT,
            constants.TURNING_ENCODER_CHANNEL_B_FRONT_LEFT,
        )

        self.swerve_module_front_right = SwerveModule(
            constants.DRIVE_MOTOR_CHANNEL_FRONT_RIGHT,
            constants.TURNING_MOTOR_CHANNEL_FRONT_RIGHT,
            constants.DRIVE_ENCODER_CHANNEL_A_FRONT_RIGHT,
            constants.DRIVE_ENCODER_CHANNEL_B_FRONT_RIGHT,
            constants.TURNING_ENCODER_CHANNEL_A_FRONT_RIGHT,
            constants.TURNING_ENCODER_CHANNEL_B_FRONT_RIGHT,
        )

        self.swerve_module_rear_left = SwerveModule(
            constants.DRIVE_MOTOR_CHANNEL_REAR_LEFT,
            constants.TURNING_MOTOR_CHANNEL_REAR_LEFT,
            constants.DRIVE_ENCODER_CHANNEL_A_REAR_LEFT,
            constants.DRIVE_ENCODER_CHANNEL_B_REAR_LEFT,
            constants.TURNING_ENCODER_CHANNEL_A_REAR_LEFT,
            constants.TURNING_ENCODER_CHANNEL_B_REAR_LEFT,
        )

        self.swerve_module_rear_right = SwerveModule(
            constants.DRIVE_MOTOR_CHANNEL_REAR_RIGHT,
            constants.TURNING_MOTOR_CHANNEL_REAR_RIGHT,
            constants.DRIVE_ENCODER_CHANNEL_A_REAR_RIGHT,
            constants.DRIVE_ENCODER_CHANNEL_B_REAR_RIGHT,
            constants.TURNING_ENCODER_CHANNEL_A_REAR_RIGHT,
            constants.TURNING_ENCODER_CHANNEL_B_REAR_RIGHT,
        )

        self.kinematics = SwerveDrive4Kinematics(
            constants.FRONT_LEFT_LOCATION,
            constants.FRONT_RIGHT_LOCATION,
            constants.REAR_LEFT_LOCATION,
            constants.REAR_RIGHT_LOCATION,
        )

    def drive(self, forward_speed: float, strafe_speed: float, turn_speed: float) -> None:
        chassis_speeds = ChassisSpeeds(forward_speed, strafe_speed, turn_speed)

        # discretized_speeds = ChassisSpeeds.discretize(chassis_speeds, period_seconds)

        moduleStates = self.kinematics.toSwerveModuleStates(chassis_speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(moduleStates, constants.MAX_VELOCITY)

        self.swerve_module_front_left.setDesiredState(moduleStates[0])
        self.swerve_module_front_right.setDesiredState(moduleStates[1])
        self.swerve_module_rear_left.setDesiredState(moduleStates[2])
        self.swerve_module_rear_right.setDesiredState(moduleStates[3])
