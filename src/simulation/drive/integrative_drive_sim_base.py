from abc import ABC, abstractmethod
import numpy as np

from wpimath.system import RK4
from wpimath.kinematics import MecanumDriveWheelPositions, MecanumDriveWheelSpeeds


class IntegrativeDriveSimBase(ABC):
    def __init__(self):
        self.inputs = np.zeros(4)
        self.state = np.zeros(8)  # wheel positions + speeds
    
    @abstractmethod
    def derivatives(self, wheel_speeds: np.ndarray, motor_inputs: np.ndarray) -> np.ndarray:
        pass
    
    def _full_derivatives(self, state: np.ndarray, motor_inputs: np.ndarray) -> np.ndarray:
        wheel_speeds = state.flatten()[4:]
        wheel_accelerations = self.derivatives(wheel_speeds, motor_inputs.flatten())
        return np.concatenate([wheel_speeds, wheel_accelerations])[:, None]
    
    def step(self, dt: float, motor_inputs: list | np.ndarray):
        self.state = RK4(self._full_derivatives, self.state[:, None], np.array(motor_inputs)[:, None], dt).flatten()
        
        wheel_positions = MecanumDriveWheelPositions()
        (
            wheel_positions.frontLeft,
            wheel_positions.frontRight,
            wheel_positions.rearLeft,
            wheel_positions.rearRight,
        ) = self.state[:4]
        
        wheel_speeds = MecanumDriveWheelSpeeds(*self.state[4:].tolist())
        return wheel_positions, wheel_speeds
