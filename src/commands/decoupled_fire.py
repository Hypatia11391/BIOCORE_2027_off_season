from typing import override, NamedTuple
import math
import numpy as np

from commands2 import Command
from wpilib import Timer
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Translation3d, Translation2d
from wpimath.kinematics import ChassisSpeeds

import src.commands.operation_constants as operate_consts
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter
import src.subsystems.mechanisms.shooter_constants as shooter_consts
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum

from src.navx.navx import Navx

GRAVITY = 9.81
LAUNCHER_ANGLE = math.pi / 3  # TODO: remeasure
SHOOTER_RADIUS = 0.05  # TODO: remeasure
FIRE_DURATION = 3
TAN_SQRD = math.tan(LAUNCHER_ANGLE)**2
SHOT_TOLERANCE = 0.5
MAX_ATTEMPT_LENGTH = 15


class DecoupledFireCommand(Command):
    def __init__(self, target: Translation3d, feed: Feed, kicker: Kicker, shooter: Shooter, drive: DriveTrainMecanum, pose_estimator: MecanumDrivePoseEstimator3d, navx: Navx):
        self.target = target
        
        self.feed = feed
        self.kicker = kicker
        self.shooter = shooter
        self.drive = drive
        self.pose_estimator = pose_estimator
        self.navx = navx
        
        self.addRequirements(feed, kicker, shooter, drive)
    
    class CaculationResult(NamedTuple):
        rpm: float
        angle: float
        is_possible: bool
    
    @staticmethod  # TODO: Fix target z discrepencey
    def calculate_rpm(relative_target: Translation3d, chassis_speed: ChassisSpeeds) -> CaculationResult:  # Calculation: https://www.desmos.com/calculator/dmf6cva5ma
        coefficients = [
            (1 / 4) * (GRAVITY**2),
            0,
            GRAVITY * relative_target.Z() - TAN_SQRD * (chassis_speed.vx**2 + chassis_speed.vy**2),
            2 * TAN_SQRD * (relative_target.X() * chassis_speed.vx + relative_target.Y() * chassis_speed.vy),
            relative_target.Z()**2 - TAN_SQRD * (relative_target.X()**2 + relative_target.Y()**2),
        ]
        
        roots = np.roots(coefficients)
        valid_roots = [r.real for r in roots if abs(r.imag) < 1e-5 and 0 < r.real]
        
        if not valid_roots:
            return DecoupledFireCommand.CaculationResult(rpm=0, angle=0, is_possible=False)
        
        t = valid_roots[0]  # TODO: add shot optimization (check for vertical velocity and basket angle)
        v = math.sqrt((relative_target.X() / t - chassis_speed.vx)**2 + (relative_target.Y() / t - chassis_speed.vy)**2) / math.cos(LAUNCHER_ANGLE)
        rpm = v / (2 * math.pi * SHOOTER_RADIUS) * 2 * 60
        angle = math.degrees(math.atan2(relative_target.Y() / t - chassis_speed.vy, relative_target.X() / t - chassis_speed.vx))
        
        if not 0 < rpm < shooter_consts.SHOOTER_FREE_SPEED:
            return DecoupledFireCommand.CaculationResult(rpm=0, angle=0, is_possible=False)
        
        return DecoupledFireCommand.CaculationResult(rpm=rpm, angle=angle, is_possible=True)
    
    def calculate_current_shot(self, rpm: float, angle: float, chassis_speed: ChassisSpeeds, relative_target_height: float) -> Translation2d:
        v = rpm * (2 * math.pi * SHOOTER_RADIUS) / 2 / 60
        t = (v * math.sin(LAUNCHER_ANGLE) + math.sqrt((v * math.sin(LAUNCHER_ANGLE))**2 - 2 * GRAVITY * relative_target_height)) / GRAVITY
        return Translation2d(t * (v * math.cos(LAUNCHER_ANGLE) * math.cos(math.radians(angle)) + chassis_speed.vx), t * (v * math.cos(LAUNCHER_ANGLE) * math.sin(math.radians(angle)) + chassis_speed.vy))
    
    @override
    def initialize(self):
        self.finished = False
        self.time_at_target_speed = -1
        self.time_attempting_shot = Timer.getFPGATimestamp()
    
    @override
    def execute(self):
        if self.time_at_target_speed < 0:
            relative_target = self.target - self.pose_estimator.getEstimatedPosition().translation()
            target_state = self.calculate_rpm(relative_target, self.navx.get_chassis_speeds())
            
            if not target_state.is_possible:
                self.feed.stop()
                self.kicker.stop()
                return
            
            current_shot = self.calculate_current_shot(self.shooter.get_left_rpm(), self.pose_estimator.getEstimatedPosition().rotation().angle_degrees, self.navx.get_chassis_speeds(), relative_target.Z())
            
            left_shooter_speed, right_shooter_speed = target_state.rpm, target_state.rpm
            
            self.shooter.set_target_rpm(left_shooter_speed, right_shooter_speed)
        
        if self.time_at_target_speed > 0 or relative_target.toTranslation2d().distance(current_shot) < SHOT_TOLERANCE:
            self.kicker.set_kicker_speed(operate_consts.KICKER_POWER)
            self.feed.set_feed_speed(operate_consts.FEED_POWER)
            
            if self.time_at_target_speed < 0:
                self.time_at_target_speed = Timer.getFPGATimestamp()
            
            elif Timer.getFPGATimestamp() - self.time_at_target_speed > FIRE_DURATION:
                self.finished = True
        
        else:
            self.feed.stop()
            self.kicker.stop()
        
        if Timer.getFPGATimestamp() - self.time_attempting_shot > MAX_ATTEMPT_LENGTH:
            self.finished = True
    
    @override
    def isFinished(self) -> bool:
        return self.finished
    
    @override
    def end(self, interrupted: bool):
        self.feed.stop()
        self.kicker.stop()
        self.shooter.stop()
