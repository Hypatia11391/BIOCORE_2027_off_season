from typing import override

from commands2 import Command
from wpilib import Timer
from wpimath.estimator import MecanumDrivePoseEstimator3d
from wpimath.geometry import Translation2d

import src.commands.operation_constants as operate_consts
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter


class ShootCommand(Command):
    def __init__(self, feed: Feed, kicker: Kicker, shooter: Shooter, pose_estimator: MecanumDrivePoseEstimator3d):
        self.feed = feed
        self.kicker = kicker
        self.shooter = shooter
        self.pose_estimator = pose_estimator

        self.addRequirements(shooter)

        self.time_at_target_speed = -1.0

    @staticmethod
    def caculate_rmp(dist: float) -> float:
        return 0  # TODO: Add shooter code

    @override
    def initialize(self):
        pass

    @override
    def execute(self):
        dist = self.pose_estimator.getEstimatedPosition().toPose2d().translation().distance(Translation2d(11.901424, 4.034536))  # TODO: Currently only supports blue team

        left_shooter_speed = self.caculate_rmp(dist)  # rt_shoot * operate_consts.HIGH_LEFT_RPM
        right_shooter_speed = left_shooter_speed  # rt_shoot * operate_consts.HIGH_RIGHT_RPM

        self.shooter.set_target_rpm(left_shooter_speed, right_shooter_speed)

        if self.shooter.is_at_target_rpm():
            self.kicker.set_kicker_speed(operate_consts.KICKER_POWER)
            self.feed.set_feed_speed(operate_consts.FEED_POWER)

            if self.time_at_target_speed < 0.0:
                self.time_at_target_speed = Timer.getFPGATimestamp()

        else:
            self.feed.stop()
            self.kicker.stop()
            # self.shooter.stop()

    @override
    def end(self, interrupted: bool):
        self.feed.stop()
        self.kicker.stop()
        self.shooter.stop()
