import numpy as np
import scipy.optimize as optim

from tests.constant_tuning.simulation.drive.simulation_drive_cost import drive_cost


def tune_drive_constants(control, robot):
    def cost_callback(params):
        kV, kA, kS = np.exp(params)
        cost = drive_cost(control, robot, kV=kV, kA=kA, kS=kS)
        print(f"Ran {kV=} {kA=} {kS=}; {cost=}")
        return cost
    
    # result = optim.minimize(
    #     cost_callback,
    #     x0 = np.log([0.8, 0.8, 0.8]),
    #     method = 'Powell',
    #     options = {'xatol': 0.00001},
    # )

    # result = optim.least_squares(
    #     cost_callback,
    #     x0 = np.log([0.8, 0.8, 0.8]),
    #     bounds = (np.log([0.0001, 0.0001, 0.0001]), np.log([10, 10, 10])),
    #     xtol = 0.00001,
    # )

    result = optim.basinhopping(
        cost_callback,
        x0 = np.log([0.81, 0.81, 0.81]),
        minimizer_kwargs = {'method': 'L-BFGS-B', 'bounds': [(np.log(0.0001), np.log(10))]*3},
        niter = 15,
        stepsize = 0.4,
        T = 1.0,
    )

    return np.exp(result.x)


def test_tune_drive_constants(control, robot):
    with control.run_robot():
        best_kV, best_kA, best_kS = tune_drive_constants(control, robot)
        print(f"{best_kV=} {best_kA=} {best_kS=}")
