from typing import override, NamedTuple
import math

from commands2 import Command
from wpilib import Timer
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Translation3d

import src.commands.operation_constants as operate_consts
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter
import src.subsystems.mechanisms.shooter_constants as shooter_consts
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum

GRAVITY = 9.81
LAUNCHER_ANGLE = math.pi / 3  # TODO: remeasure
SHOOTER_RADIUS = 0.05  # TODO: remeasure
FIRE_DURATION = 3


class FireCommand(Command):
    def __init__(self, target: Translation3d, feed: Feed, kicker: Kicker, shooter: Shooter, drive: DriveTrainMecanum, pose_estimator: MecanumDrivePoseEstimator3d):
        self.target = target

        self.feed = feed
        self.kicker = kicker
        self.shooter = shooter
        self.drive = drive
        self.pose_estimator = pose_estimator

        self.addRequirements(feed, kicker, shooter, drive)

    class CaculationResult(NamedTuple):
        rpm: float
        is_possible: bool

    @staticmethod
    def calculate_rpm(relative_target: Translation3d) -> CaculationResult:  # Calculation: https://www.desmos.com/calculator/y9ouqrg1f2
        if math.atan2(relative_target.Z(), relative_target.toTranslation2d().norm()) >= LAUNCHER_ANGLE:
            return FireCommand.CaculationResult(rpm=0, is_possible=False)

        t = math.sqrt(2 * (math.tan(LAUNCHER_ANGLE) * relative_target.toTranslation2d().norm() - relative_target.Z()) / GRAVITY)
        v = relative_target.toTranslation2d().norm() / (math.cos(LAUNCHER_ANGLE) * t)
        rpm = v / (2 * math.pi * SHOOTER_RADIUS) * 2 * 60

        if rpm > shooter_consts.SHOOTER_FREE_SPEED:
            return FireCommand.CaculationResult(rpm=rpm, is_possible=False)

        entry_angle = math.atan2(math.sin(LAUNCHER_ANGLE) * v - GRAVITY * t, relative_target.toTranslation2d().norm() / t)

        if entry_angle > -math.pi / 4:
            return FireCommand.CaculationResult(rpm=rpm, is_possible=False)

        return FireCommand.CaculationResult(rpm=rpm, is_possible=True)

    @override
    def initialize(self):
        relative_target = self.target - self.pose_estimator.getEstimatedPosition().translation()
        self.calculation_result = self.calculate_rpm(relative_target)

        if self.calculation_result.is_possible:
            self.finished = False
        else:
            self.finished = True

        self.time_at_target_speed = -1.0

    @override
    def execute(self):
        left_shooter_speed, right_shooter_speed = self.calculation_result.rpm, self.calculation_result.rpm

        self.shooter.set_target_rpm(left_shooter_speed, right_shooter_speed)

        if self.shooter.is_at_target_rpm() or self.time_at_target_speed > 0.0:
            self.kicker.set_kicker_speed(operate_consts.KICKER_POWER)
            self.feed.set_feed_speed(operate_consts.FEED_POWER)

            if self.time_at_target_speed < 0.0:
                self.time_at_target_speed = Timer.getFPGATimestamp()

            elif Timer.getFPGATimestamp() - self.time_at_target_speed > FIRE_DURATION:
                self.finished = True

        else:
            self.feed.stop()
            self.kicker.stop()

    @override
    def isFinished(self) -> bool:
        return self.finished

    @override
    def end(self, interrupted: bool):
        self.feed.stop()
        self.kicker.stop()
        self.shooter.stop()
