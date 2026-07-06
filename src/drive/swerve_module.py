import drive_train_constants as constants

from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpilib import Encoder
import rev

from math import pi


class SwerveModule:
    def __init__(
        self,
        drive_motor_channel: int,
        turning_motor_channel: int,
        drive_encoder_channel_a: int,
        drive_encoder_channel_b: int,
        turning_encoder_channel_a: int,
        turning_encoder_channel_b: int,
    ):
        # self.drive_motor_channel = drive_motor_channel
        # self.turning_motor_channel = turning_motor_channel
        # self.drive_encoder_channel_a = drive_encoder_channel_a
        # self.drive_encoder_channel_b = drive_encoder_channel_b
        # self.turning_encoder_channel_a = turning_encoder_channel_a
        # self.turning_encoder_channel_b = turning_encoder_channel_b

        self.drive_PID_controller = PIDController(1, 0, 0)

        self.turning_PID_controller = ProfiledPIDController(
            1,
            0,
            0,
            TrapezoidProfile.Constraints(
                constants.MAX_ANGULAR_VELOCITY,
                constants.MAX_ANGULAR_ACCELERATION,
            ),
        )

        self.drive_feed_forward = SimpleMotorFeedforwardMeters(1, 3)
        self.turn_feed_forward = SimpleMotorFeedforwardRadians(1, 0.5)

        self.drive_motor = rev.SparkMax(drive_motor_channel, rev.SparkLowLevel.MotorType.kBrushless)
        self.turning_motor = rev.SparkMax(turning_motor_channel, rev.SparkLowLevel.MotorType.kBrushless)

        self.drive_encoder = Encoder(drive_encoder_channel_a, drive_encoder_channel_b)
        self.turning_encoder = Encoder(turning_encoder_channel_a, turning_encoder_channel_b)

        self.turning_PID_controller.enableContinuousInput(-pi, pi)

    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        encoder_rotation = Rotation2d(self.turning_encoder.getDistance())

        desired_state.optimize(encoder_rotation)

        desired_state.cosineScale(encoder_rotation)

        # Volatge
        drive_output = self.drive_PID_controller.calculate(self.drive_encoder.getRate(), desired_state.speed)

        # Volatge
        drive_feed_forward = self.drive_feed_forward.calculate(desired_state.speed)

        # Volatge
        turn_output = self.turning_PID_controller.calculate(self.turning_encoder.getDistance(), desired_state.angle.radians())

        # Volatge
        turn_feed_forward = self.turn_feed_forward.calculate(self.turning_PID_controller.getSetpoint().velocity)

        self.drive_motor.setVoltage(drive_output + drive_feed_forward)
        self.turning_motor.setVoltage(turn_output + turn_feed_forward)
