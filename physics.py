from math import pi

from pyfrc.physics.core import PhysicsInterface


class PhysicsEngine: 
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.drive = robot.robot_container.drive
    
    def update_sim(self, now: float, tm_diff: float):
        self.physics_controller.drive(self.drive.get_relative_speeds(), tm_diff)
