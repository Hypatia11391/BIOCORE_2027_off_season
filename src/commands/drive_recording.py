from typing import override
from pathlib import Path

from commands2 import Command

from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum
from src.recording.recording import Recording


class DriveRecording(Command):
    def __init__(self, drive: DriveTrainMecanum, recording: Recording):
        super().__init__()

        self.drive = drive
        self.recording = recording

        self.addRequirements(self.drive)
    
    @override
    def initialize(self):
        pass

    @override
    def execute(self):
        frame = self.recording.next_frame()
        name = 'applied-outputs' if 'applied-outputs' in frame else 'set-values'
        self.drive.drive_from_raw_motor_set_values(
            fl_percent = frame[name][0],
            fr_percent = frame[name][1],
            rl_percent = frame[name][2],
            rr_percent = frame[name][3],
        )