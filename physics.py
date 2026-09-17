from math import pi

import wpilib
from wpilib.simulation import SimDeviceSim, AnalogGyroSim
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
from wpimath.kinematics import MecanumDriveWheelSpeeds
from rev import SparkMaxSim, SparkLowLevel
from wpimath.system.plant import DCMotor

from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_RIGHT_ID, REAR_LEFT_ID, REAR_RIGHT_ID, MAX_ANGULAR_SPEED, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO


class PhysicsEngine: 
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.drive = robot.robot_container.drive
        
        self.front_left_sim = SparkMaxSim(self.drive.left_front_drive, DCMotor.NEO(1))
        self.front_right_sim = SparkMaxSim(self.drive.right_front_drive, DCMotor.NEO(1))
        self.rear_left_sim = SparkMaxSim(self.drive.left_rear_drive, DCMotor.NEO(1))
        self.rear_right_sim = SparkMaxSim(self.drive.right_rear_drive, DCMotor.NEO(1))
        
        self.gyro_sim = AnalogGyroSim(0)
    
    def update_sim(self, now: float, tm_diff: float):
        # Get wheel speeds in radians per second
        fl_radps = self.drive.left_front_drive.get() * MAX_ANGULAR_SPEED
        fr_radps = self.drive.right_front_drive.get() * MAX_ANGULAR_SPEED
        rl_radps = self.drive.left_rear_drive.get() * MAX_ANGULAR_SPEED
        rr_radps = self.drive.right_rear_drive.get() * MAX_ANGULAR_SPEED
        #print(f'{fl_radps=}, {fr_radps=}, {rl_radps=}, {rr_radps=}')
        
        # Set velocity (rpm)
        radps_to_rpm = 60 / (2*pi)  
        voltage = wpilib.RobotController.getBatteryVoltage()      
        self.front_left_sim.iterate(fl_radps * radps_to_rpm, voltage, tm_diff)
        self.front_right_sim.iterate(fr_radps * radps_to_rpm, voltage, tm_diff)
        self.rear_left_sim.iterate(rl_radps * radps_to_rpm, voltage, tm_diff)
        self.rear_right_sim.iterate(rr_radps * radps_to_rpm, voltage, tm_diff)
        
        # # Advance position (rotations per simualtion frame)
        # radps_to_rpf = tm_diff / (2*pi)
        # self.front_left_sim.setPosition(self.front_left_sim.getPosition() + fl_radps * radps_to_rpf)
        # self.front_right_sim.setPosition(self.front_right_sim.getPosition() + fr_radps * radps_to_rpf)
        # self.rear_left_sim.setPosition(self.rear_left_sim.getPosition() + rl_radps * radps_to_rpf)
        # self.rear_right_sim.setPosition(self.rear_right_sim.getPosition() + rr_radps * radps_to_rpf)
        
        # Compute wheel speeds (m/s), chassis speeds and drive simulation
        radps_to_mps = (1/(2*pi)) * WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE
        wheel_speeds = MecanumDriveWheelSpeeds(
            fl_radps * radps_to_mps,
            fr_radps * radps_to_mps,
            rl_radps * radps_to_mps,
            rr_radps * radps_to_mps
        )
        chassis_speeds = self.drive.kinematics.toChassisSpeeds(wheel_speeds)
        self.physics_controller.drive(chassis_speeds, tm_diff)
        
        # Advance simulated gyro heading
        self.gyro_sim.setAngle(self.gyro_sim.getAngle() + chassis_speeds.omega*tm_diff*(180/pi))  # convert from rad/s to deg/frame
