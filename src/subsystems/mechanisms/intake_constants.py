from rev import SparkBaseConfig, FeedbackSensor

from typing import Final

INTAKE_FEED_ID: Final[int] = 4
INTAKE_LIFT_ID: Final[int] = 6

INTAKE_LIFT_MIN_POS: Final[float] = -51.0
INTAKE_LIFT_MAX_POS: Final[float] = -10.0
INTAKE_LIFT_START_POS: Final[float] = 0.0

INTAKE_LIFT_INVERTED: Final[bool] = True
INTAKE_LIFT_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kBrake
INTAKE_LIFT_SMART_LIMIT: Final[int] = 40
INTAKE_LIFT_VOLTAGE_COMPENSATION: Final[float] = 12
INTAKE_LIFT_ENCODER_VELOCITY_CONVERSION_FACTOR: Final[float] = 1
INTAKE_LIFT_ENCODER_POSITION_CONVERSION_FACTOR: Final[float] = 1
INTAKE_LIFT_CLOSED_LOOP_FEEDBACK_SENSOR: Final[FeedbackSensor] = FeedbackSensor.kPrimaryEncoder

INTAKE_LIFT_kP: Final[float] = 50
INTAKE_LIFT_kI: Final[float] = 0
INTAKE_LIFT_kD: Final[float] = 5

INTAKE_FEED_INVERTED: Final[bool] = True
INTAKE_FEED_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kBrake
INTAKE_FEED_SMART_LIMIT: Final[int] = 40
INTAKE_FEED_VOLTAGE_COMPENSATION: Final[float] = 12
