from math import pi

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import VelocityVoltage, PositionVoltage

import src.subsystems.drive.swerve_constants as consts


class SwerveModule:
    def __init__(self, drive_id: int, turn_id: int, cancoder_id: int, cancoder_offset: float):
        # self.drive_motor_channel = drive_motor_channel
        # self.turning_motor_channel = turning_motor_channel
        # self.drive_encoder_channel_a = drive_encoder_channel_a
        # self.drive_encoder_channel_b = drive_encoder_channel_b
        # self.turning_encoder_channel_a = turning_encoder_channel_a
        # self.turning_encoder_channel_b = turning_encoder_channel_b
        
        self.drive_motor = TalonFX(drive_id)
        self.turn_motor = TalonFX(turn_id)
        
        self.cancoder = CANcoder(cancoder_id)
        
        self.config_motors(cancoder_offset)
        
        self.drive_velocity_requester = VelocityVoltage(0.0)
        self.turn_position_requester = PositionVoltage(0.0)
        
        self.drive_velocity_signal = self.drive_motor.get_velocity()
        self.turn_position_signal = self.turn_motor.get_position()
    
    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        self.turn_position_signal.refresh(report_error=True)
        
        desired_state.optimize(Rotation2d.fromRotations(self.turn_position_signal.value))
        
        self.drive_motor.set_control(self.drive_velocity_requester.with_velocity(desired_state.speed))
        self.turn_motor.set_control(self.turn_position_requester.with_position(desired_state.angle.radians() / (2.0 * pi)))
    
    def config_motors(self, cancoder_offset: float) -> None:
        cancoder_config = CANcoderConfiguration()  # TODO: might need more config, cw vs ccw and sensor range
        cancoder_config.magnet_sensor.magnet_offset = cancoder_offset
        
        self.cancoder.configurator.apply(cancoder_config)
        
        drive_config = TalonFXConfiguration()
        
        drive_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        drive_config.feedback.sensor_to_mechanism_ratio = consts.DRIVE_GEAR_RATIO / (2.0 * pi * consts.WHEEL_RADIUS)
        
        drive_config.slot0.k_p = consts.DRIVE_K_P
        drive_config.slot0.k_i = consts.DRIVE_K_I
        drive_config.slot0.k_d = consts.DRIVE_K_D
        
        drive_config.slot0.k_s = consts.DRIVE_K_S
        drive_config.slot0.k_v = consts.DRIVE_K_V
        drive_config.slot0.k_a = consts.DRIVE_K_A
        
        self.drive_motor.configurator.apply(drive_config)
        
        turn_config = TalonFXConfiguration()
        
        turn_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turn_config.feedback.sensor_to_mechanism_ratio = consts.TURN_GEAR_RATIO
        turn_config.feedback.feedback_remote_sensor_id = self.cancoder.device_id
        
        turn_config.slot0.k_p = consts.TURN_K_P
        turn_config.slot0.k_i = consts.TURN_K_I
        turn_config.slot0.k_d = consts.TURN_K_D
        
        turn_config.slot0.k_s = consts.TURN_K_S
        turn_config.slot0.k_v = consts.TURN_K_V
        turn_config.slot0.k_a = consts.TURN_K_A
        
        self.turn_motor.configurator.apply(turn_config)
    
    def get_state(self) -> SwerveModuleState:
        self.drive_velocity_signal.refresh(report_error=True)
        self.turn_position_signal.refresh(report_error=True)
        
        return SwerveModuleState(self.drive_velocity_signal.value, Rotation2d.fromRotations(self.turn_position_signal.value))
