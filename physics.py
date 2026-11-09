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
    
    def update_sim(self, now: float, tm_diff: float):
        self.physics_controller.drive(self.drive.get_relative_speeds(), tm_diff)