from typing import Final

from rev import FeedbackSensor, SparkBaseConfig

INTAKE_FEED_ID: Final[int] = 4
INTAKE_LIFT_ID: Final[int] = 6

# INTAKE_LIFT_DOWN_POS: Final[float] = -12.0  # ] TODO: once have more accurate method of testing position add back
# INTAKE_LIFT_UP_POS: Final[float] = -10.0  # ] NOTE are these backwards maybe? this is what makes it move at startup without any controller input, so it being backwards could explain why motor is stalling
# INTAKE_LIFT_START_POS: Final[float] = 0.0

INTAKE_LIFT_INVERTED: Final[bool] = True
INTAKE_LIFT_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kBrake
INTAKE_LIFT_SMART_LIMIT: Final[int] = 40
INTAKE_LIFT_VOLTAGE_COMPENSATION: Final[float] = 12
INTAKE_LIFT_ENCODER_VELOCITY_CONVERSION_FACTOR: Final[float] = 1  # 360 / (48 * (50 / 18))
INTAKE_LIFT_ENCODER_POSITION_CONVERSION_FACTOR: Final[float] = 1  # 360 / (48 * (50 / 18))  # Conversion to degrees devide
INTAKE_LIFT_CLOSED_LOOP_FEEDBACK_SENSOR: Final[FeedbackSensor] = FeedbackSensor.kPrimaryEncoder
INTAKE_LIFT_POSITION_THRESHOLD: Final[float] = 1

INTAKE_LIFT_kP: Final[float] = 0.03
INTAKE_LIFT_kI: Final[float] = 0  # 0.0005
INTAKE_LIFT_kD: Final[float] = 0.05

INTAKE_FEED_INVERTED: Final[bool] = True
INTAKE_FEED_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kBrake
INTAKE_FEED_SMART_LIMIT: Final[int] = 40
INTAKE_FEED_VOLTAGE_COMPENSATION: Final[float] = 12
