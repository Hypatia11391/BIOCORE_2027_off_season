from typing import override

from commands2 import Command
from wpimath.controller import PIDController
from wpimath.estimator import MecanumDrivePoseEstimator3d

from src.subsystems.drive.drive_handler import DriveHandler, ClaimCommand, DriveTurnAndTranslationCommand

ANGLE_THRESHOLD = 3  # TODO: Tune


class TurnToCommand(Command):
    def __init__(self, target_angle: float, drive: DriveHandler, pose_estimator: MecanumDrivePoseEstimator3d, pid: PIDController, manual: bool = False):
        self.target_angle = target_angle
        self.manual = manual
        
        self.drive = drive
        self.pose_estimator = pose_estimator
        self.pid = pid
        
        self.addRequirements(self.drive.get_dummy_translation(), self.drive.get_dummy_turn())
        
        self.id = self.drive.get_id()
        
        self.drive.push_command(ClaimCommand(self.id, True, True))
        
        if not self.manual:
            self.pid.enableContinuousInput(-180.0, 180.0)
    
    @override
    def initialize(self):
        pass
    
    @override
    def execute(self):
        pid_out = self.pid.calculate(
            self.pose_estimator.getEstimatedPosition().rotation().angle_degrees,
            self.target_angle,
        )
        
        pid_out = min(1, max(-1, pid_out))
        
        self.drive.push_command(DriveTurnAndTranslationCommand(self.id, 0, 0, pid_out))
    
    @override
    def end(self, interrupted: bool):
        pass
    
    @override
    def isFinished(self) -> bool:
        angle = self.pose_estimator.getEstimatedPosition().rotation().angle_degrees
        
        return abs(angle - self.target_angle) < ANGLE_THRESHOLD
