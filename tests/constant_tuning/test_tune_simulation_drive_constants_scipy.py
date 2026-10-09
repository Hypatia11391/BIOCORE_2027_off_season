from scipy.optimize import least_squares, differential_evolution

from src.constants import STARTING_POSE

FPS = 50
kA_TOLERANCE = 0.000001

def cost_kA(control, robot, kA):
    print(f'Costing {kA=}...')
    
    robot.robot_container.drive.reset_simulation(kA)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)

    for _ in range(1*FPS):
        control.step_timing(
            seconds=1/FPS,
            autonomous=False,
            enabled=True,
        )

    robot.robot_container.drive.reset_simulation(kA)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)
    
    while True:
        control.step_timing(
            seconds=1/FPS,
            autonomous=True,
            enabled=True,
        )
        speeds = robot.robot_container.drive.get_relative_speeds()
        if speeds.vx<0.0001 and speeds.vy<0.0001 and speeds.omega<0.00005:
          break

    cost = robot.robot_container.drive.get_pose_rms()
    print(f'{cost=}')
    return cost

def test_tune_simulation_constants(control, robot):
    print('############## Tuning kA...')
    with control.run_robot():
        kA = differential_evolution(
            lambda params: cost_kA(control, robot, params[0]),
            [0.25], # initial guess
            bounds=([0],[0.5]),
            verbose=True,
        )
    print(f'best {kA=}')
