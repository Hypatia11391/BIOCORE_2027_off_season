from dataclasses import dataclass

import numpy as np
from wpimath.geometry import Translation2d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelSpeeds
from wpimath.system import RKDP, LinearSystem_1_1_1


@dataclass
class MecanumDriveSimSpec:
    gear_ratio: float = 1
    moi: float = 0
    mass: float = 0
    wheel_radius: float = 0
    wheel_positions: tuple[Translation2d, Translation2d, Translation2d, Translation2d] = (
        Translation2d(0, 0),
        Translation2d(0, 0),
        Translation2d(0, 0),
        Translation2d(0, 0),
    )
    max_volts: float = 12


class StateEnum:
    x = 0
    y = 1
    theta = 2
    v_fl = 3
    v_fr = 4
    v_rl = 5
    v_rr = 6
    p_fl = 7
    p_fr = 8
    p_rl = 9
    p_rr = 10


class MecanumDriveSim:
    def __init__(
        self,
        plants: tuple[LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1, LinearSystem_1_1_1],
        spec: MecanumDriveSimSpec,
    ):
        self.plants = plants
        self.spec = spec

        self.state = np.zeros((11,))
        self.inputs = np.zeros((11,))

        self.kinematics = MecanumDriveKinematics(
            self.spec.wheel_positions[0],
            self.spec.wheel_positions[1],
            self.spec.wheel_positions[2],
            self.spec.wheel_positions[3],
        )

    def set_inputs(self, fl_input: float, fr_input: float, rl_input: float, rr_input: float) -> None:
        self.inputs = np.array([fl_input, fr_input, rl_input, rr_input])
        self.inputs = np.clip(self.inputs, -self.spec.max_volts, self.spec.max_volts)

    def update(self, dt: float) -> None:
        self.state = RKDP(
            lambda state, inputs: self.get_state_derivitave_vector(np.asarray(state), np.asarray(inputs)),
            self.state,
            self.inputs,
            dt,
        )

    def get_state_derivitave_vector(self, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        assert state.shape == (11, 1), "Passed in state to get state derivitave vector is not the right shape"
        assert inputs.shape == (4, 1), "Passed in inputs to get state derivitave vector is not the right shape"

        gear_ratio_sq = self.spec.gear_ratio * self.spec.gear_ratio

        A = np.zeros((8, 8))
        B = np.zeros((8, 4))

        for i in range(4):
            A[i, i] = self.plants[i].A()[0] * self.spec.gear_ratio
            B[i, i] = self.plants[i].B()[0] * gear_ratio_sq

        A[4:8, 0:4] = np.eye(4)

        v_fl = state[StateEnum.v_fl][0]
        v_fr = state[StateEnum.v_fr][0]
        v_rl = state[StateEnum.v_rl][0]
        v_rr = state[StateEnum.v_rr][0]

        chassis_speeds = self.kinematics.toChassisSpeeds(MecanumDriveWheelSpeeds(v_fl, v_fr, v_rl, v_rr))

        state_derivitave_vector = np.zeros(11)

        theta = state[StateEnum.theta][0]
        state_derivitave_vector[StateEnum.x] = chassis_speeds.vx * np.cos(theta) - chassis_speeds.vy * np.sin(theta)
        state_derivitave_vector[StateEnum.y] = chassis_speeds.vx * np.sin(theta) + chassis_speeds.vy * np.cos(theta)
        state_derivitave_vector[StateEnum.theta] = chassis_speeds.omega
        state_derivitave_vector[3:11] = (A @ state[3:11] + B @ inputs).flatten()

        return state_derivitave_vector

    def reset(self) -> None:
        self.state = np.zeros(11)
        self.inputs = np.zeros(4)

    def get_fl_position(self) -> float:
        return self.state[StateEnum.p_fl][0]

    def get_fr_position(self) -> float:
        return self.state[StateEnum.p_fr][0]

    def get_rl_position(self) -> float:
        return self.state[StateEnum.p_rl][0]

    def get_rr_position(self) -> float:
        return self.state[StateEnum.p_rr][0]

    def get_fl_velocity(self) -> float:
        return self.state[StateEnum.v_fl][0]

    def get_fr_velocity(self) -> float:
        return self.state[StateEnum.v_fr][0]

    def get_rl_velocity(self) -> float:
        return self.state[StateEnum.v_rl][0]

    def get_rr_velocity(self) -> float:
        return self.state[StateEnum.v_rr][0]
