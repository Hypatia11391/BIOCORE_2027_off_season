from typing import override
from math import pi

import wpilib
from commands2 import Subsystem
from rev import PersistMode, ResetMode, SparkBase, SparkLowLevel, SparkMax, SparkMaxConfig, SparkMaxSim
from wpimath.system.plant import LinearSystemId, DCMotor
from wpilib.simulation import LinearSystemSim_1_1_1

import src.subsystems.mechanisms.shooter_constants as shooter_consts
from src.network_server.network_server import NetworkServer


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.motor_left = SparkMax(shooter_consts.SHOOTER_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
        self.motor_right = SparkMax(shooter_consts.SHOOTER_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)

        self.config_shooter_motor(self.motor_left, shooter_consts.SHOOTER_LEFT_INVERTED)
        self.config_shooter_motor(self.motor_right, shooter_consts.SHOOTER_RIGHT_INVERTED)

        self.left_encoder = self.motor_left.getEncoder()
        self.right_encoder = self.motor_right.getEncoder()

        self.left_loop = self.motor_left.getClosedLoopController()
        self.right_loop = self.motor_right.getClosedLoopController()

        self.target_rpm_left = 0
        self.target_rpm_right = 0

        if wpilib.RobotBase.isSimulation():
            self._init_simulation()

    def _init_simulation(self):
        self.battery_voltage = wpilib.RobotController.getBatteryVoltage()
        plant = LinearSystemId.identifyVelocitySystemRadians(
            kV = self.battery_voltage / (shooter_consts.SHOOTER_MAX_SPEED * (2*pi)/60),
            kA = 0.01,
        )
        
        self.system_sim = LinearSystemSim_1_1_1(plant)
        
        self.motor_left_sim = SparkMaxSim(self.motor_left, DCMotor.NEO(1))
        self.motor_right_sim = SparkMaxSim(self.motor_right, DCMotor.NEO(1))
        
        self.last_sim_time = wpilib.Timer.getFPGATimestamp()
    
    def set_target_rpm(self, target_rpm_left: float, target_rpm_right: float) -> None:
        self.target_rpm_left = target_rpm_left
        self.target_rpm_right = target_rpm_right

        self.left_loop.setSetpoint(self.target_rpm_left, SparkBase.ControlType.kVelocity)
        self.right_loop.setSetpoint(self.target_rpm_right, SparkBase.ControlType.kVelocity)

    def get_left_rpm(self) -> float:
        return self.left_encoder.getVelocity()

    def get_right_rpm(self) -> float:
        return self.right_encoder.getVelocity()

    def is_at_target_rpm(self) -> bool:
        # print(f"{abs(self.get_left_rpm() - self.target_rpm_left) <= shooter_consts.SHOOTER_RPM_TOLERANCE=} and {abs(self.get_right_rpm() - self.target_rpm_right) <= shooter_consts.SHOOTER_RPM_TOLERANCE=}")
        return abs(self.get_left_rpm() - self.target_rpm_left) <= shooter_consts.SHOOTER_RPM_TOLERANCE and abs(self.get_right_rpm() - self.target_rpm_right) <= shooter_consts.SHOOTER_RPM_TOLERANCE

    def stop(self) -> None:
        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self.motor_left.stopMotor()
        self.motor_right.stopMotor()

    def config_shooter_motor(self, motor: SparkMax, inverted: bool) -> None:
        config = SparkMaxConfig()
        config.inverted(inverted)
        config.setIdleMode(shooter_consts.SHOOTER_IDLE_MODE)
        config.smartCurrentLimit(shooter_consts.SHOOTER_SMART_LIMIT)
        config.voltageCompensation(shooter_consts.SHOOTER_VOLTAGE_COMPENSATION)
        config.closedLoopRampRate(shooter_consts.SHOOTER_CLOSED_LOOP_RAMP_RATE)

        config.encoder.velocityConversionFactor(shooter_consts.SHOOTER_VELOCITY_CONVERSION_FACTOR)

        config.closedLoop.setFeedbackSensor(shooter_consts.SHOOTER_FEEDBACK_SENSOR)
        config.closedLoop.pid(shooter_consts.SHOOTER_KP, shooter_consts.SHOOTER_KI, shooter_consts.SHOOTER_KD)
        config.closedLoop.outputRange(shooter_consts.SHOOTER_OUTPUT_RANGE_MIN, shooter_consts.SHOOTER_OUTPUT_RANGE_MAX)

        config.closedLoop.feedForward.kV(shooter_consts.SHOOTER_KV)

        motor.configureAsync(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    def get_left_voltage(self) -> float:
        return self.motor_left.getBusVoltage() * self.motor_left.getAppliedOutput()

    def get_right_voltage(self) -> float:
        return self.motor_right.getBusVoltage() * self.motor_right.getAppliedOutput()

    @override
    def periodic(self) -> None:
        NetworkServer.getInstance().set_float("shooter-left-rpm", self.left_encoder.getVelocity())
        NetworkServer.getInstance().set_float("shooter-left-target-rpm", self.target_rpm_left)
        NetworkServer.getInstance().set_float("shooter-right-rpm", self.right_encoder.getVelocity())
        NetworkServer.getInstance().set_float("shooter-right-target-rpm", self.target_rpm_right)

    @override
    def simulationPeriodic(self):
        current_time = wpilib.Timer.getFPGATimestamp()
        tm_diff = current_time - self.last_sim_time
        self.last_sim_time = current_time

        in_power = (self.motor_left.get() + self.motor_right.get())
        self.system_sim.setInput(0, in_power * self.battery_voltage)
        self.system_sim.update(tm_diff)
        out_rpm = self.system_sim.getOutput(0) * 60/(2*pi)
        self.motor_left_sim.iterate(out_rpm, self.battery_voltage, tm_diff)
        self.motor_right_sim.iterate(out_rpm, self.battery_voltage, tm_diff)
