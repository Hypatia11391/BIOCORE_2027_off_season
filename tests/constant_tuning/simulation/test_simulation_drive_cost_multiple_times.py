import os

from tests.constant_tuning.simulation.simulation_drive_cost import drive_cost


def test_simulation_drive_cost_multiple_times(control, robot):
    with control.run_robot():
        for i in range(3):
            cost = drive_cost(control, robot, kA=float(os.environ.get("kA",0)), kV=float(os.environ.get("kV",0)) or None)
            print(f'{i}th time {cost=}')
