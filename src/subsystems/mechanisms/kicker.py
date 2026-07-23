from commands2 import Subsystem
from rev import SparkMax, SparkMaxConfig, SparkLowLevel, ResetMode, PersistMode

import src.subsystems.mechanisms.kicker_constants as kicker_consts


class Kicker(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor = SparkMax(kicker_consts.KICKER_ID, SparkLowLevel.MotorType.kBrushed)

        config = SparkMaxConfig()
        config.inverted(kicker_consts.KICKER_INVERTED)
        config.setIdleMode(kicker_consts.KICKER_IDLE_MODE)
        config.smartCurrentLimit(kicker_consts.KICKER_SMART_LIMIT)
        config.voltageCompensation(kicker_consts.KICKER_VOLTAGE_COMPENSATION)

        self.motor.configureAsync(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    def set_kicker_speed(self, speed: float) -> None:
        """
        Speed should be between -1, and 1
        """

        self.motor.set(speed)

    def stop(self) -> None:
        self.motor.stopMotor()
