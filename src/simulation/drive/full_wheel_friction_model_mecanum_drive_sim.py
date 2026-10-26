from typing import override
from dataclasses import dataclass
import numpy as np

from wpimath.system import LinearSystem_1_1_1
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelSpeeds

from src.simulation.drive.integrative_drive_sim_base import IntegrativeDriveSimBase


@dataclass
class FullWheelFrictionModelMecanumParameters:
    # static friction (4,)
    rolling_static_friction: np.ndarray
    sliding_static_friction: np.ndarray
    slipping_rolling_static_friction: np.ndarray
    slipping_sliding_static_friction: np.ndarray

    # viscous friction (4,)
    rolling_viscous_friction: np.ndarray
    sliding_viscous_friction: np.ndarray
    slipping_rolling_viscous_friction: np.ndarray
    slipping_sliding_viscous_friction: np.ndarray

    # wheel traction (4,)
    rolling_traction: np.ndarray
    sliding_traction: np.ndarray

    # other parameters
    location: np.ndarray  # (4,2)
    centre_of_mass: np.ndarray  # (2,)
    mass: float
    roller_angle: float  # rad


class FullWheelFrictionModelMecanumDriveSim(IntegrativeDriveSimBase):
    @override
    def __init__(self, parameters: FullWheelFrictionModelMecanumParameters, kinematics: MecanumDriveKinematics):
        self.params = parameters
        self.kinematics = kinematics
    
    def _wheel_vel_p(self, wheel_speeds):
        chassis_speeds = self.kinematics.toChassisSpeeds(MecanumDriveWheelSpeeds(*wheel_speeds))
        c,s = np.cos(self.params.roller_angle), np.sin(self.params.roller_angle)
        rotvel = np.array([[-s,-c],[c,-s]]) @ self.params.location
        vel = np.array([[chassis_speeds.vy, chassis_speeds.vx]]) + rotvel
        vel_p = np.dot(np.stack([c,s], axis=0), axis=0)

    def derivatives(self, wheel_speeds: np.ndarray, motor_inputs: np.ndarray):
        vel_p = self._wheel_vel_p(wheel_speeds)
