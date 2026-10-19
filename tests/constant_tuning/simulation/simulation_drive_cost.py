import os

from wpimath.kinematics import ChassisSpeeds

from src.constants import STARTING_POSE, FPS
from tests.util import run_for, RobotState


def drive_cost(control, robot, kA=None, kV=None):
    print(f"{kA=} {kV=}")
    robot.robot_container.drive.reset_simulation(kA, kV)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)
    robot.robot_container.drive.robot_drive.setSafetyEnabled(False)
    if hasattr(robot, "test_command"):
        robot.test_command.recording.reset()
    # control.step_timing()
    # run_for(control, FPS, RobotState.DISABLED, time=1)

    # print('pose:',robot.robot_container.drive.get_pose_2d())
    # print('speed:',robot.robot_container.drive.get_relative_speeds())

    cost = 0
    for _ in run_for(control, FPS, RobotState.TEST, time=10, iter_=True):
        cost += robot.robot_container.drive.get_speeds_rms(ChassisSpeeds(*robot.test_command.recording.current_frame()["chassis-speeds"])) ** 2
        if robot.test_command.recording.is_finished():
            break

    return cost


def test_drive_cost(control, robot):
    with control.run_robot():
        cost = drive_cost(control, robot, kA=float(os.environ.get("kA", 0)) or None, kV=float(os.environ.get("kV", 0)) or None)
        print(f"{cost=}")
