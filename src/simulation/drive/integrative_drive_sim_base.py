from abc import ABC, abstractmethod
import numpy as np

from wpimath.system import RK4
from wpimath.geometry import Rotation2d
from wpimath.kinematics import MecanumDriveWheelSpeeds, MecanumDriveKinematics, ChassisSpeeds


class DriveState:
    def __init__(self, state: np.ndarray | None = None):
        self.state_np = np.zeros(14) if state is None else state.flatten()
    
    def wheel_positions(self):
        return self.state_np[0:4]
    
    def pose(self):
        return self.state_np[4:7]
    
    def yaw(self):
        return self.state_np[6]
    
    def wheel_speeds(self):
        return self.state_np[7:11]
    
    def field_oriented_chassis_speeds(self):
        return self.state_np[11:14]
    
    def robot_relative_chassis_speeds(self):
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds(*self.field_oriented_chassis_speeds()), self.yaw())
        return np.array([speeds.vx, speeds.vy, speeds.omega])
    
    def speeds(self):
        return self.state_np[7:14]
    
    def as_matrix(self):
        return self.state_np[:, None]


class IntegrativeDriveSimBase(ABC):
    def __init__(self, kinematics: MecanumDriveKinematics | None = None):
        self.inputs = np.zeros(4)
        self.state = DriveState()
        self.kinematics = kinematics
    
    @abstractmethod
    def derivatives(self, state: DriveState, inputs: np.ndarray) -> np.ndarray:
        pass
    
    def _full_derivatives(self, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        drive_state = DriveState(state)
        accel = self.derivatives(drive_state, inputs.flatten())

        if self.kinematics is not None and accel.size == 4:
            chassis_speeds = self.kinematics.toChassisSpeeds(MecanumDriveWheelSpeeds(*drive_state.wheel_speeds()))
            chassis_accel = self.kinematics.toChassisSpeeds(MecanumDriveWheelSpeeds(*accel))
            field_accel = ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds(
                    chassis_accel.vx - chassis_speeds.vy * chassis_speeds.omega,  # ] Centripetal force
                    chassis_accel.vy + chassis_speeds.vx * chassis_speeds.omega,  # ]
                    chassis_accel.omega,
                ),
                Rotation2d(drive_state.yaw()),
            )
            accel = np.concatenate([accel, [field_accel.vx, field_accel.vy, field_accel.omega]])
        assert accel.size == 7
        
        return np.concatenate([drive_state.speeds(), accel])[:, None]
    
    def step(self, dt: float, motor_inputs: list | np.ndarray):
        self.state = DriveState(RK4(self._full_derivatives, self.state.as_matrix(), np.array(motor_inputs)[:, None], dt))
