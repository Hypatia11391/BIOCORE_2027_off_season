from typing import override
from math import pi

import wpilib
from commands2 import Subsystem
from rev import PersistMode, ResetMode, SparkLowLevel, SparkMax, SparkMaxConfig, SparkMaxSim
from wpimath.system.plant import LinearSystemId, DCMotor
from wpilib.simulation import LinearSystemSim_1_1_1

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

        if wpilib.RobotBase.isSimulation():
            self._init_simulation()

    def _init_simulation(self):
        self.battery_voltage = wpilib.RobotController.getBatteryVoltage()
        plant = LinearSystemId.identifyVelocitySystemRadians(
            kV = self.battery_voltage / (feed_consts.FEED_MAX_SPEED * (2*pi)/60),
            kA = 0.01,
        )
        self.system_sim = LinearSystemSim_1_1_1(plant)
        self.motor_sim = SparkMaxSim(self.motor, DCMotor.NEO(1))
        self.last_sim_time = wpilib.Timer.getFPGATimestamp()

    def set_feed_speed(self, speed: float) -> None:
        """
        Speed should be between -1, and 1
        """

        self.power = speed

        self.motor.set(speed)

    def stop(self) -> None:
        self.set_feed_speed(0)
        self.motor.stopMotor()

    @override
    def periodic(self) -> None:
        NetworkServer.getInstance().set_float("feed-power", self.power)

    @override
    def simulationPeriodic(self):
        current_time = wpilib.Timer.getFPGATimestamp()
        tm_diff = current_time - self.last_sim_time
        self.last_sim_time = current_time
        
        self.system_sim.setInput(0, self.motor.get() * self.battery_voltage)
        self.system_sim.update(tm_diff)
        self.motor_sim.iterate(self.system_sim.getOutput(0) * 60/(2*pi), self.battery_voltage, tm_diff)
