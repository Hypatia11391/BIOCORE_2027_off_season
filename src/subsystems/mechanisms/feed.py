from typing import override

from commands2 import Subsystem
from rev import PersistMode, ResetMode, SparkLowLevel, SparkMax, SparkMaxConfig

import src.subsystems.mechanisms.feed_constants as feed_consts
from src.network_server.network_server import NetworkServer


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

        self.power = 0

    def set_feed_speed(self, speed: float) -> None:
        """
        Speed should be between -1, and 1
        """

        self.power = speed

        self.motor.set(speed)

    def stop(self) -> None:
        self.motor.stopMotor()

    @override
    def periodic(self) -> None:
        NetworkServer.getInstance().set_float("feed-power", self.power)
