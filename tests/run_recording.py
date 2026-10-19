from pyfrc.test_support.controller import TestController

from robot import Robot


def test(control: TestController, robot: Robot):
    with control.run_robot():
        robot.run_recording("output.json")
        control.step_timing(seconds=100.0, autonomous=True, enabled=True)
        print(robot.robot_container.pose_estimator.getEstimatedPosition())
