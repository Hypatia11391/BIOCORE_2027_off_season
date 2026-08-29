from commands2 import Subsystem
from rev import PersistMode, ResetMode, SparkBase, SparkLowLevel, SparkMax, SparkMaxConfig

import src.subsystems.mechanisms.shooter_constants as shooter_consts


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor_left = SparkMax(shooter_consts.SHOOTER_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
        self.motor_right = SparkMax(shooter_consts.SHOOTER_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)

        self.config_shooter_motor(self.motor_left, shooter_consts.SHOOTER_LEFT_INVERTED)
        self.config_shooter_motor(self.motor_right, shooter_consts.SHOOTER_RIGHT_INVERTED)

        self.left_encoder = self.motor_left.getEncoder()
        self.right_encoder = self.motor_right.getEncoder()

        self.left_loop = self.motor_left.getClosedLoopController()
        self.right_loop = self.motor_right.getClosedLoopController()

        self.target_rpm_left = 0
        self.target_rpm_right = 0

    def set_target_rpm(self, target_rpm_left: float, target_rpm_right: float) -> None:
        self.target_rpm_left = target_rpm_left
        self.target_rpm_right = target_rpm_right

        self.left_loop.setSetpoint(self.target_rpm_left, SparkBase.ControlType.kVelocity)
        self.right_loop.setSetpoint(self.target_rpm_right, SparkBase.ControlType.kVelocity)

    def get_left_rpm(self) -> float:
        return self.left_encoder.getVelocity()

    def get_right_rpm(self) -> float:
        return self.right_encoder.getVelocity()

    def is_at_target_rpm(self) -> bool:
        # print(f"{abs(self.get_left_rpm() - self.target_rpm_left) <= shooter_consts.SHOOTER_RPM_TOLERANCE=} and {abs(self.get_right_rpm() - self.target_rpm_right) <= shooter_consts.SHOOTER_RPM_TOLERANCE=}")
        return abs(self.get_left_rpm() - self.target_rpm_left) <= shooter_consts.SHOOTER_RPM_TOLERANCE and abs(self.get_right_rpm() - self.target_rpm_right) <= shooter_consts.SHOOTER_RPM_TOLERANCE

    def stop(self) -> None:
        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self.motor_left.stopMotor()
        self.motor_right.stopMotor()

    def config_shooter_motor(self, motor: SparkMax, inverted: bool) -> None:
        config = SparkMaxConfig()
        config.inverted(inverted)
        config.setIdleMode(shooter_consts.SHOOTER_IDLE_MODE)
        config.smartCurrentLimit(shooter_consts.SHOOTER_SMART_LIMIT)
        config.voltageCompensation(shooter_consts.SHOOTER_VOLTAGE_COMPENSATION)
        config.closedLoopRampRate(shooter_consts.SHOOTER_CLOSED_LOOP_RAMP_RATE)

        config.encoder.velocityConversionFactor(shooter_consts.SHOOTER_VELOCITY_CONVERSION_FACTOR)

        config.closedLoop.setFeedbackSensor(shooter_consts.SHOOTER_FEEDBACK_SENSOR)
        config.closedLoop.pid(shooter_consts.SHOOTER_KP, shooter_consts.SHOOTER_KI, shooter_consts.SHOOTER_KD)
        config.closedLoop.outputRange(shooter_consts.SHOOTER_OUTPUT_RANGE_MIN, shooter_consts.SHOOTER_OUTPUT_RANGE_MAX)

        config.closedLoop.feedForward.kV(shooter_consts.SHOOTER_KV)

        motor.configureAsync(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    def get_left_voltage(self) -> float:
        return self.motor_left.getBusVoltage() * self.motor_left.getAppliedOutput()

    def get_right_voltage(self) -> float:
        return self.motor_right.getBusVoltage() * self.motor_right.getAppliedOutput()
