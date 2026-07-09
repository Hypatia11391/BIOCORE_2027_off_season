from rev import SparkBaseConfig

from typing import Final

KICKER_ID: Final[int] = 8
KICKER_INVERTED: Final[bool] = True
KICKER_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kBrake
KICKER_SMART_LIMIT: Final[int] = 40
KICKER_VOLTAGE_COMPENSATION: Final[float] = 12
