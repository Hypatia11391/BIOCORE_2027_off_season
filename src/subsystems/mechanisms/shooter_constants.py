from typing import Final

from rev import SparkBaseConfig, FeedbackSensor

SHOOTER_LEFT_ID: Final[int] = 3
SHOOTER_RIGHT_ID: Final[int] = 5
SHOOTER_LEFT_INVERTED: Final[bool] = True
SHOOTER_RIGHT_INVERTED: Final[bool] = False
SHOOTER_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kCoast
SHOOTER_SMART_LIMIT: Final[int] = 40
SHOOTER_VOLTAGE_COMPENSATION: Final[float] = 12.0
SHOOTER_CLOSED_LOOP_RAMP_RATE: Final[float] = 0.25
SHOOTER_VELOCITY_CONVERSION_FACTOR: Final[float] = 1.0
SHOOTER_FEEDBACK_SENSOR: Final[FeedbackSensor] = FeedbackSensor.kPrimaryEncoder
SHOOTER_OUTPUT_RANGE_MIN: Final[float] = -1.0
SHOOTER_OUTPUT_RANGE_MAX: Final[float] = 1.0
SHOOTER_FREE_SPEED: Final[float] = 5676.0
SHOOTER_KV: Final[float] = 1 / SHOOTER_FREE_SPEED
SHOOTER_RPM_TOLERANCE: Final[float] = 400.0

SHOOTER_KP: Final[float] = 0.00006
SHOOTER_KI: Final[float] = 0.0
SHOOTER_KD: Final[float] = 0.1
