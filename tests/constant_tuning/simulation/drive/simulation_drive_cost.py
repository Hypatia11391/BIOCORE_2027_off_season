import os

from wpimath.kinematics import ChassisSpeeds

from src.constants import STARTING_POSE, FPS
from tests.util import run_for, RobotState

SPEEDS_NAMES = {'chassis-speeds', 'robotVelocity'}


def drive_cost(control, robot, kV=None, kA=None, kS=None):
    robot.robot_container.drive.reset_simulation(kV=kV, kA=kA, kS=kS)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)
    robot.robot_container.drive.robot_drive.setSafetyEnabled(False)
    if robot.test_command is not None: robot.test_command.reset()
    
    cost = 0
    for _ in run_for(control, FPS, RobotState.TEST, iter_=True):
        frame = robot.test_command.recording.current_frame()
        name = next(iter(SPEEDS_NAMES & frame.keys()))
        speeds = ChassisSpeeds(*robot.test_command.recording.current_frame()[name])
        cost += robot.robot_container.drive.get_speeds_rms(speeds)**2
        if robot.test_command.recording.is_finished():
            break
    
    return cost


def test_drive_cost(control, robot):
    with control.run_robot():
        cost = drive_cost(
            control,
            robot,
            kV=float(os.environ.get("kV", 0)) or None,
            kA=float(os.environ.get("kA", 0)) or None,
            kS=float(os.environ.get("kS", 0)) or None,
        )
        print(f"{cost=}")
