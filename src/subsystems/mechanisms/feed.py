from commands2 import Subsystem
from rev import SparkMax, SparkMaxConfig, SparkLowLevel, ResetMode, PersistMode

import src.subsystems.mechanisms.feed_constants as feed_consts


class Feed(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor = SparkMax(feed_consts.FEED_ID, SparkLowLevel.MotorType.kBrushless)

        config = SparkMaxConfig()
        config.inverted(feed_consts.FEED_INVERTED)
        config.setIdleMode(feed_consts.FEED_IDLE_MODE)
        config.smartCurrentLimit(feed_consts.FEED_SMART_LIMIT)
        config.voltageCompensation(feed_consts.FEED_VOLTAGE_COMPENSATION)

        self.motor.configureAsync(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    def set_feed_speed(self, speed: float) -> None:
        """
        Speed should be between -1, and 1
        """

        self.motor.set(speed)

    def stop(self) -> None:
        self.motor.stopMotor()
