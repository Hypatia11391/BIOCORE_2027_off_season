from math import pi

import wpilib
from wpilib.simulation import SimDeviceSim, LinearSystemSim_1_1_1
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import MecanumDriveWheelSpeeds
from rev import SparkMaxSim
from wpimath.system.plant import DCMotor, LinearSystemId

from src.subsystems.drive.drive_train_constants import WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO, MAX_SPEED


kA = 0.0007


class PhysicsEngine: 
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.drive = robot.robot_container.drive
        self.battery_voltage = wpilib.RobotController.getBatteryVoltage()
        print(self.battery_voltage)

        wheel_plant = LinearSystemId.identifyVelocitySystemRadians(
            kV = self.battery_voltage / (MAX_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_GEAR_RATIO * 60),
            kA = kA,
        )

        print(self.battery_voltage / (MAX_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_GEAR_RATIO * 60))
        
        self.fl_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        self.fr_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        self.rl_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        self.rr_system_sim = LinearSystemSim_1_1_1(wheel_plant)
        
        self.fl_motor_sim = SparkMaxSim(self.drive.left_front_drive, DCMotor.NEO(1))
        self.fr_motor_sim = SparkMaxSim(self.drive.right_front_drive, DCMotor.NEO(1))
        self.rl_motor_sim = SparkMaxSim(self.drive.left_rear_drive, DCMotor.NEO(1))
        self.rr_motor_sim = SparkMaxSim(self.drive.right_rear_drive, DCMotor.NEO(1))
        
        self.navx_sim = SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx_sim.getDouble("Yaw")
    
    def update_sim(self, now: float, tm_diff: float):

        # Update wheel linear system
        self.fl_system_sim.setInput(0, self.drive.left_front_drive.get() * self.battery_voltage)
        self.fr_system_sim.setInput(0, self.drive.right_front_drive.get() * self.battery_voltage)
        self.rl_system_sim.setInput(0, self.drive.left_rear_drive.get() * self.battery_voltage)
        self.rr_system_sim.setInput(0, self.drive.right_rear_drive.get() * self.battery_voltage)

        self.fl_system_sim.update(tm_diff)
        self.fr_system_sim.update(tm_diff)
        self.rl_system_sim.update(tm_diff)
        self.rr_system_sim.update(tm_diff)

        fl_rpm = self.fl_system_sim.getOutput(0)
        fr_rpm = self.fr_system_sim.getOutput(0)
        rl_rpm = self.rl_system_sim.getOutput(0)
        rr_rpm = self.rr_system_sim.getOutput(0)
        
        # Update encoders
        # Rev library is apparently horrible so we need to multiply by the velocity conversion factor manually and hope it matches the position conversion factor
        mps_to_rpm = WHEEL_GEAR_RATIO / WHEEL_CIRCUMFERENCE * 60
        voltage = wpilib.RobotController.getBatteryVoltage()
        conversion_factor = self.drive.left_front_drive.configAccessor.encoder.getVelocityConversionFactor()
        
        self.fl_motor_sim.iterate(fl_rpm * conversion_factor, self.battery_voltage, tm_diff)
        self.fr_motor_sim.iterate(fr_rpm * conversion_factor, self.battery_voltage, tm_diff)
        self.rl_motor_sim.iterate(rl_rpm * conversion_factor, self.battery_voltage, tm_diff)
        self.rr_motor_sim.iterate(rr_rpm * conversion_factor, self.battery_voltage, tm_diff)
        
        # Update physics_controller
        wheel_speeds = MecanumDriveWheelSpeeds(
            fl_rpm * conversion_factor,
            fr_rpm * conversion_factor,
            rl_rpm * conversion_factor,
            rr_rpm * conversion_factor,
        )
        chassis_speeds = self.drive.kinematics.toChassisSpeeds(wheel_speeds)
        self.physics_controller.drive(chassis_speeds, tm_diff)
        
        # Update simulated navx gyro heading
        # We convert from rad/s to deg/frame
        # wpilib is CCW positive, navx is CW positive, so we subtract
        self.navx_yaw.set(self.navx_yaw.get() - chassis_speeds.omega*tm_diff*(180/pi))
