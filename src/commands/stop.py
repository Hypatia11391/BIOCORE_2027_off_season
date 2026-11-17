from typing import override

from commands2 import Command

from src.subsystems.drive.drive_handler import DriveHandler, ClaimCommand, DriveTurnAndTranslationCommand
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.intake import Intake
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter

SPEED_THRESHOLD = 0.05  # TODO: Tune


class StopCommand(Command):
    def __init__(self, feed: Feed, kicker: Kicker, shooter: Shooter, drive: DriveHandler, intake: Intake):
        self.feed = feed
        self.kicker = kicker
        self.shooter = shooter
        self.drive = drive
        self.intake = intake
        
        self.addRequirements(
            self.feed,
            self.kicker,
            self.shooter,
            self.drive.get_dummy_translation(),
            self.drive.get_dummy_turn(),
            self.intake,
        )
        
        self.id = self.drive.get_id()
        
        self.drive.push_command(ClaimCommand(self.id, True, True))
    
    @override
    def initialize(self):
        pass
    
    @override
    def execute(self):
        self.feed.stop()
        self.kicker.stop()
        self.shooter.stop()
        self.drive.push_command(DriveTurnAndTranslationCommand(self.id, 0, 0, 0))
        self.intake.stop()
    
    @override
    def end(self, interrupted: bool):
        pass
    
    @override
    def isFinished(self) -> bool:
        wheel_speeds = self.drive.drive.get_wheel_speeds()  # TODO fix
        
        return all([
            abs(wheel_speeds.frontLeft) < SPEED_THRESHOLD,
            abs(wheel_speeds.frontRight) < SPEED_THRESHOLD,
            abs(wheel_speeds.rearLeft) < SPEED_THRESHOLD,
            abs(wheel_speeds.rearRight) < SPEED_THRESHOLD,
        ])
