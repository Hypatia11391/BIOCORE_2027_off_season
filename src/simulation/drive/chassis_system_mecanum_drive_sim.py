from typing import override

from src.simulation.drive.integrative_drive_sim_base import IntegrativeDriveSimBase, DriveState


class ChassisSystemMecanumDriveSim(IntegrativeDriveSimBase):
    @override
    def __init__(self):
        pass
    
    @override
    def derivatives(self, state: DriveState, inputs: np.ndarray):
        linear_thrust = self._chassis_speeds_mat @ state.robot_relative_chassis_speeds() + self._wheel_speeds_mat @ state.wheel_speeds() + self._inputs_mat @ inputs
        static_friction = np.where(...)
