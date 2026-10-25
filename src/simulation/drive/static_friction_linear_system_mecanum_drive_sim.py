from typing import override
import numpy as np

from wpimath.system import LinearSystem_1_1_1

from src.simulation.drive.integrative_drive_sim_base import IntegrativeDriveSimBase


class StaticFrictionLinearSystemMecanumDriveSim(IntegrativeDriveSimBase):
    @override
    def __init__(self, A: np.ndarray, B: np.ndarray, kS: float|np.ndarray, kS_threshold: float|np.ndarray = 0.01):
        super().__init__()
        self.A = A
        self.B = B
        self.kS = kS
        self.kS_threshold = kS_threshold
    
    @classmethod
    def from_1d_plants(cls, plants: tuple[LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1], kS: float|np.ndarray, kS_threshold: float|np.ndarray = 0.01):
        A = np.diag([p.A().item() for p in plants])
        B = np.diag([p.B().item() for p in plants])
        return cls(A, B, kS, kS_threshold)
    
    @override
    def derivatives(self, wheel_speeds: np.ndarray, motor_inputs: np.ndarray) -> np.ndarray:
        moving = np.abs(wheel_speeds) > self.kS_threshold
        kS_sign = np.sign(np.where(moving, wheel_speeds, motor_inputs))
        inputs = motor_inputs - self.kS * kS_sign
        inputs = np.where(  # set to zero if sign flipped
            moving | (np.abs(motor_inputs) > self.kS),
            inputs,
            0
        )
        return (self.A @ wheel_speeds + self.B @ inputs)
