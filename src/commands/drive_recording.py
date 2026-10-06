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
        self.drive.drive_from_applied_outputs(
            fl_percent = frame['applied-outputs'][0]*100,
            fr_percent = frame['applied-outputs'][1]*100,
            rl_percent = frame['applied-outputs'][2]*100,
            rr_percent = frame['applied-outputs'][3]*100,
        )