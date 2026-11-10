from typing import override
import numpy as np

from wpimath.system import LinearSystem_1_1_1
from wpimath.kinematics import MecanumDriveKinematics

from src.simulation.drive.integrative_drive_sim_base import IntegrativeDriveSimBase, DriveState


class StaticFrictionLinearSystemMecanumDriveSim(IntegrativeDriveSimBase):
    @override
    def __init__(self, A: np.ndarray, B: np.ndarray, kS: float | np.ndarray, kinematics: MecanumDriveKinematics, kS_threshold: float | np.ndarray = 0.01):
        super().__init__(kinematics=kinematics)
        self.A = A
        self.B = B
        self.kS = kS
        self.kS_threshold = kS_threshold
    
    @classmethod
    def from_1d_plants(cls, plants: tuple[LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1], kS: float | np.ndarray, kinematics: MecanumDriveKinematics, kS_threshold: float | np.ndarray = 0.01):
        A = np.diag([p.A().item() for p in plants])
        B = np.diag([p.B().item() for p in plants])
        return cls(A, B, kS, kinematics=kinematics, kS_threshold=kS_threshold)
    
    @override
    def derivatives(self, state: DriveState, inputs: np.ndarray) -> np.ndarray:
        wheel_speeds = state.wheel_speeds()
        moving = np.abs(wheel_speeds) > self.kS_threshold
        kS_sign = np.sign(np.where(moving, wheel_speeds, inputs))
        reduced_inputs = inputs - self.kS * kS_sign
        reduced_inputs = np.where(  # set to zero if sign flipped
            moving | (np.abs(inputs) > self.kS),
            reduced_inputs,
            0,
        )
        return (self.A @ wheel_speeds + self.B @ reduced_inputs)
