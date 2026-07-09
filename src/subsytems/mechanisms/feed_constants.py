from rev import SparkBaseConfig

from typing import Final

FEED_ID: Final[int] = 7
FEED_SPEED_MULTIPLIER: Final[float] = 0.5
FEED_INVERTED: Final[bool] = False
FEED_IDLE_MODE: Final[SparkBaseConfig.IdleMode] = SparkBaseConfig.IdleMode.kCoast
FEED_SMART_LIMIT: Final[int] = 40
FEED_VOLTAGE_COMPENSATION: Final[float] = 12
