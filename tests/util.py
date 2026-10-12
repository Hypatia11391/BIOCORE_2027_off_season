from enum import Enum
import itertools

from wpilib.simulation import DriverStationSim


class RobotState(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTONOMOUS = 2
    TEST = 3

def run_for(control, fps: float, robot_state: RobotState, time: float|None = None, iter_: bool = False):
    try:
        DriverStationSim.setTest(robot_state==RobotState.TEST)
        for _ in (itertools.count() if time is None else range(int(time*fps))):
            control.step_timing(
                seconds = 1/fps,
                enabled = robot_state!=RobotState.DISABLED,
                autonomous = robot_state==RobotState.AUTONOMOUS,
            )
            if iter_:
                yield
    finally:
        DriverStationSim.setTest(False)
