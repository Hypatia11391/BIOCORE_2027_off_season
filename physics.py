from math import pi

import wpilib
from wpilib.simulation import SimDeviceSim, AnalogGyroSim
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
from wpimath.kinematics import MecanumDriveWheelSpeeds
from rev import SparkMaxSim, SparkLowLevel
from wpimath.system.plant import DCMotor

from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_RIGHT_ID, REAR_LEFT_ID, REAR_RIGHT_ID, MAX_ANGULAR_SPEED, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO, MAX_SPEED


class PhysicsEngine: 
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.drive = robot.robot_container.drive
        
        self.front_left_sim = SparkMaxSim(self.drive.left_front_drive, DCMotor.NEO(1))
        self.front_right_sim = SparkMaxSim(self.drive.right_front_drive, DCMotor.NEO(1))
        self.rear_left_sim = SparkMaxSim(self.drive.left_rear_drive, DCMotor.NEO(1))
        self.rear_right_sim = SparkMaxSim(self.drive.right_rear_drive, DCMotor.NEO(1))
        
        self.navx_sim = SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx_sim.getDouble("Yaw")
    
    def update_sim(self, now: float, tm_diff: float):
        fl = self.drive.left_front_drive.get()
        fr = self.drive.right_front_drive.get()
        rl = self.drive.left_rear_drive.get()
        rr = self.drive.right_rear_drive.get()
        print(f'{fl=}, {fr=}, {rl=}, {rr=}')
        
        # Get wheel speeds in radians per second
        fl_mps = self.drive.left_front_drive.get() * MAX_SPEED
        fr_mps = self.drive.right_front_drive.get() * MAX_SPEED
        rl_mps = self.drive.left_rear_drive.get() * MAX_SPEED
        rr_mps = self.drive.right_rear_drive.get() * MAX_SPEED
        print(f'{fl_mps=}, {fr_mps=}, {rl_mps=}, {rr_mps=}')
        for motor in (self.drive.left_front_drive, self.drive.right_front_drive, self.drive.left_rear_drive, self.drive.right_rear_drive): assert abs(motor.configAccessor.encoder.getPositionConversionFactor() / 60 - motor.configAccessor.encoder.getVelocityConversionFactor()) < 1e-9  # close enough for floating point errors
        
        # Iterate encoders
        # Rev library is apparently horrible so we need to multiply by the velocity conversion factor manually and hope it matches the position conversion factor
        # 1. Calculate constants
        mps_to_rpm = WHEEL_GEAR_RATIO / WHEEL_CIRCUMFERENCE * 60
        voltage = wpilib.RobotController.getBatteryVoltage()

        # 2. Check that position and velocity conversion factors match
        # they must be close enough for floating point errors
        for motor in (self.drive.left_front_drive, self.drive.right_front_drive, self.drive.left_rear_drive, self.drive.right_rear_drive): assert abs(motor.configAccessor.encoder.getPositionConversionFactor() / 60 - motor.configAccessor.encoder.getVelocityConversionFactor()) < 1e-9, "Position and velocity conversion factors must have a ratio of 60"

        # 3. Actually call iterate four times
        self.front_left_sim.iterate(fl_mps * mps_to_rpm * self.drive.left_front_drive.configAccessor.encoder.getVelocityConversionFactor(), voltage, tm_diff)
        self.front_right_sim.iterate(fr_mps * mps_to_rpm * self.drive.right_front_drive.configAccessor.encoder.getVelocityConversionFactor(), voltage, tm_diff)
        self.rear_left_sim.iterate(rl_mps * mps_to_rpm * self.drive.left_rear_drive.configAccessor.encoder.getVelocityConversionFactor(), voltage, tm_diff)
        self.rear_right_sim.iterate(rr_mps * mps_to_rpm * self.drive.right_rear_drive.configAccessor.encoder.getVelocityConversionFactor(), voltage, tm_diff)
        print(f'{fl_mps*mps_to_rpm=}')
        
        # Compute wheel speeds (m/s), chassis speeds and drive simulation
        wheel_speeds = MecanumDriveWheelSpeeds(fl_mps, fr_mps, rl_mps, rr_mps)
        #print(f'physics.py, {wheel_speeds=}')
        chassis_speeds = self.drive.kinematics.toChassisSpeeds(wheel_speeds)
        #print(f'{chassis_speeds=}')
        self.physics_controller.drive(chassis_speeds, tm_diff)
        
        # Advance simulated gyro heading
        # We convert from rad/s to deg/frame
        # wpilib is CCW positive, navx is CW positive, so we subtract
        self.navx_yaw.set(self.navx_yaw.get() - chassis_speeds.omega*tm_diff*(180/pi))
