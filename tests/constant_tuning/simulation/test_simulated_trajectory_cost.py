import os

from wpimath.kinematics import ChassisSpeeds

from src.constants import STARTING_POSE
from tests.util import run_for, RobotState


FPS = 50

def get_trajectory_cost(control, robot, kA=None, kV=None):
    robot.robot_container.drive.reset_simulation(kA, kV)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)
    robot.robot_container.drive.robot_drive.setSafetyEnabled(False)

    cost = 0
    for _ in run_for(control, FPS, RobotState.TEST, iter_=True):
        cost += robot.robot_container.drive.get_speeds_rms(ChassisSpeeds(*robot.test_command.recording.current_frame()['chassis-speeds']))**2
        if robot.test_command.recording.is_finished(): break
    
    return cost

def test(control, robot):
    with control.run_robot():
        cost = get_trajectory_cost(control, robot, kA=float(os.environ.get("kA",0)) or None, kV=float(os.environ.get("kV",0)) or None)
        print(f'{cost=}')