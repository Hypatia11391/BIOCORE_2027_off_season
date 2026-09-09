from typing import override
from wpilib import Timer
from commands2 import Command

from src.autonomous_driving.autonomous_driver import AutonomousDriver


class DriveToPoseAutonomous(Command):
    def __init__(self):
        self.driver = None
        self.last_time = None
        
    def configure(self, get_pose_2d_fn, get_relative_speeds_fn, drive_from_chassis_speeds_fn, forward_pid_consts, strafe_pid_consts, turn_pid_consts):
        self.driver = AutonomousDriver(get_pose_2d_fn, get_relative_speeds_fn, drive_from_chassis_speeds_fn, forward_pid_consts, strafe_pid_consts, turn_pid_consts)
    
    @override
    def execute(self):
        time = Timer.getFPGATimestamp()
        h = 0 if self.last_time is None else time-self.last_time
        self.driver.step(h)
        self.last_time = time