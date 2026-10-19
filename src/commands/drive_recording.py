from typing import override

from commands2 import Command

from src.recording.recording import Recording
from src.subsystems.drive.drive_train_mecanum import DriveTrainMecanum


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

        print(
            "Applied outputs: ",
            frame["applied-outputs"][0],
            frame["applied-outputs"][1],
            frame["applied-outputs"][2],
            frame["applied-outputs"][3],
        )

        self.drive.drive_from_raw_motor_set_values(
            fl_percent=frame["applied-outputs"][0],
            fr_percent=frame["applied-outputs"][1],
            rl_percent=frame["applied-outputs"][2],
            rr_percent=frame["applied-outputs"][3],
        )
