import numpy as np
import scipy.optimize as optim

from tests.constant_tuning.simulation.simulation_drive_cost import drive_cost


def tune_drive_constants(control, robot):
    def cost_callback(params):
        kA, kV = np.exp(params)
        cost = drive_cost(control, robot, kA, kV)
        print(f"Ran {kA=}, {kV=}; {cost=}")
        return cost

    result = optim.minimize(  # least_squares(
        cost_callback,
        x0=np.log([0.01, 0.1]),
        method="Powell",
        options={"xatol": 0.00001},
    )
    return np.exp(result.x)


def test_tune_drive_constants(control, robot):
    with control.run_robot():
        best_kA, best_kV = tune_drive_constants(control, robot)
        print(f"{best_kA=}")
        print(f"{best_kV=}")
