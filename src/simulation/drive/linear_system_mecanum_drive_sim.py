from typing import override
import numpy as np

from wpimath.system import LinearSystem_1_1_1

from src.simulation.drive.integrative_drive_sim_base import IntegrativeDriveSimBase


class LinearSystemMecanumDriveSim(IntegrativeDriveSimBase):
    @override
    def __init__(self, A: np.array, B: np.array):
        super().__init__()
        self.A = A
        self.B = B
    
    @classmethod
    def from_1d_plants(cls, plants: tuple[LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1]):
        A = np.diag([p.A().item() for p in plants])
        B = np.diag([p.B().item() for p in plants])
        return cls(A, B)
    
    @override
    def derivatives(self, wheel_speeds: np.ndarray, motor_inputs: np.ndarray) -> np.ndarray:
        # print(f'{wheel_speeds=}')
        # print(f'{self.A @ wheel_speeds + self.B @ motor_inputs=}')
        return (self.A @ wheel_speeds + self.B @ motor_inputs)
