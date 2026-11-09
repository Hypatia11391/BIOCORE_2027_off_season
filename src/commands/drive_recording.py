from typing import override
import os

from commands2 import Command
import wpilib

from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum
from src.recording.recording import Recording

OUTPUT_NAMES = {'applied-outputs', 'set-values', 'appliedOutput'}


class DriveRecording(Command):
    def __init__(self, drive: DriveTrainMecanum, recording: Recording, live_time: bool | None = None):
        super().__init__()
        
        self.drive = drive
        self.recording = recording
        self.live_time = live_time or os.environ.get("LIVE_TIME", None) == "true"
        
        self.start_time = wpilib.Timer.getFPGATimestamp()
        
        self.addRequirements(self.drive)
    
    @override
    def initialize(self):
        pass
    
    @override
    def execute(self):
        if self.live_time:
            time = wpilib.Timer.getFPGATimestamp() - self.start_time
            frame = self.recording.jump_to_relative_time(time)
        else:
            frame = self.recording.next_frame()
        
        name = next(iter(OUTPUT_NAMES & frame.keys()))
        self.drive.drive_from_raw_motor_set_values(
            fl_percent=frame[name][0],
            fr_percent=frame[name][1],
            rl_percent=frame[name][2],
            rr_percent=frame[name][3],
        )
    
    def reset(self):
        self.start_time = wpilib.Timer.getFPGATimestamp()
        self.recording.reset()
