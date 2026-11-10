from typing import override
import numpy as np

from wpimath.system import LinearSystem_1_1_1
from wpimath.kinematics import MecanumDriveKinematics

from src.simulation.drive.integrative_drive_sim_base import IntegrativeDriveSimBase, DriveState


class LinearSystemMecanumDriveSim(IntegrativeDriveSimBase):
    @override
    def __init__(self, A: np.array, B: np.array, kinematics: MecanumDriveKinematics):
        super().__init__(kinematics=kinematics)
        self.A = A
        self.B = B
        self.kinematics = kinematics
    
    @classmethod
    def from_1d_plants(cls, plants: tuple[LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1], kinematics: MecanumDriveKinematics):
        A = np.diag([p.A().item() for p in plants])
        B = np.diag([p.B().item() for p in plants])
        return cls(A, B, kinematics=kinematics)
    
    @override
    def derivatives(self, state: DriveState, inputs: np.ndarray) -> np.ndarray:
        return (self.A @ state.wheel_speeds() + self.B @ inputs)
