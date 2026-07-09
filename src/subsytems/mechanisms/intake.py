from commands2 import Subsystem
from rev import SparkMax, SparkMaxConfig, SparkLowLevel, ResetMode, PersistMode

import src.subsytems.mechanisms.intake_constants as intake_consts


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intake_lift = SparkMax(intake_consts.INTAKE_LIFT_ID, SparkLowLevel.MotorType.kBrushless)
        self.intake_feed = SparkMax(intake_consts.INTAKE_FEED_ID, SparkLowLevel.MotorType.kBrushless)

        self.lift_loop = self.intake_lift.getClosedLoopController()
        self.lift_encoder = self.intake_lift.getEncoder()

        self.target_pos = self.lift_encoder.getPosition()

        lift_config = SparkMaxConfig()
        lift_config.inverted(intake_consts.INTAKE_LIFT_INVERTED)
        lift_config.setIdleMode(intake_consts.INTAKE_LIFT_IDLE_MODE)
        lift_config.smartCurrentLimit(intake_consts.INTAKE_LIFT_SMART_LIMIT)
        lift_config.voltageCompensation(intake_consts.INTAKE_LIFT_VOLTAGE_COMPENSATION)
        lift_config.encoder.velocityConversionFactor(intake_consts.INTAKE_LIFT_ENCODER_VELOCITY_CONVERSION_FACTOR)
        lift_config.encoder.positionConversionFactor(intake_consts.INTAKE_LIFT_ENCODER_POSITION_CONVERSION_FACTOR)
        lift_config.closedLoop.setFeedbackSensor(intake_consts.INTAKE_LIFT_CLOSED_LOOP_FEEDBACK_SENSOR)
        lift_config.closedLoop.pid(intake_consts.INTAKE_LIFT_kP, intake_consts.INTAKE_LIFT_kI, intake_consts.INTAKE_LIFT_kD)

        self.intake_lift.configureAsync(
            lift_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        feed_config = SparkMaxConfig()
        feed_config.inverted(intake_consts.INTAKE_FEED_INVERTED)
        feed_config.setIdleMode(intake_consts.INTAKE_FEED_IDLE_MODE)
        feed_config.smartCurrentLimit(intake_consts.INTAKE_FEED_SMART_LIMIT)
        feed_config.voltageCompensation(intake_consts.INTAKE_FEED_VOLTAGE_COMPENSATION)

        self.intake_feed.configureAsync(
            feed_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    # In degrees
    def set_lift_position(self, target_pos: float) -> None:
        self.target_pos = max(intake_consts.INTAKE_LIFT_MIN_POS, min(target_pos, intake_consts.INTAKE_LIFT_MAX_POS))  # Clamp

        self.lift_loop.setSetpoint(self.target_pos / 360, SparkLowLevel.ControlType.kPosition)

    # Speed between -1, 1
    def set_feed_speed(self, speed: float) -> None:
        self.intake_feed.set(speed)

    def stop(self) -> None:
        self.intake_lift.stopMotor()
        self.intake_feed.stopMotor()
