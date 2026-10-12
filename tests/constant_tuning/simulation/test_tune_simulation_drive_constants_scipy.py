from scipy.optimize import least_squares, differential_evolution
from wpilib.simulation import DriverStationSim
from wpimath.kinematics import ChassisSpeeds

from src.constants import STARTING_POSE
from tests.util import run_for, RobotState

FPS = 50
kA_TOLERANCE = 0.000001

def cost_kA(control, robot, kA, kV=None):
    print(f'Costing {kA=} {kV=}...')

    robot.robot_container.drive.reset_simulation(kA, kV)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)
    robot.robot_container.drive.robot_drive.setSafetyEnabled(False)
    
    run_for(control, 1, FPS, RobotState.TELEOP)
    #for _ in range(1*FPS):
    #    control.step_timing(
    #        seconds=1/FPS,
    #        autonomous=False,
    #        enabled=True,
    #    )

    robot.robot_container.drive.reset_simulation(kA, kV)
    robot.robot_container.drive.reset_encoders()
    robot.robot_container.drive.reset_pose_3d(STARTING_POSE)
    robot.robot_container.drive.robot_drive.setSafetyEnabled(False)

    cost = 0

    def periodic_fn():
        cost += robot.robot_container.drive.get_speeds_rms(ChassisSpeeds(*robot.test_command.recording.current_frame()['chassis-speeds']))**2
        
        if robot.test_command.recording.i >= len(robot.test_command.recording.data)-1:
            return True
    run_for(control, 1000, FPS, RobotState.TEST, periodic_fn)

    #DriverStationSim.setTest(True)
    #
    #while True:
    #    control.step_timing(
    #        seconds=1/FPS,
    #        autonomous=False,
    #        enabled=True,
    #    )

    #    cost += robot.robot_container.drive.get_speeds_rms(ChassisSpeeds(*robot.test_command.recording.current_frame()['chassis-speeds']))**2
    #    
    #    if robot.test_command.recording.i >= len(robot.test_command.recording.data)-1:
    #        break
    #    
    #    #speeds = robot.robot_container.drive.get_relative_speeds()
    #    #if speeds.vx<0.0001 and speeds.vy<0.0001 and speeds.omega<0.00005:
    #      #break

    #DriverStationSim.setTest(False)
    
    cost = robot.robot_container.drive.get_pose_rms()
    print(f'done, f{cost=}')
    return cost

def test_tune_simulation_constants(control, robot):
    print('############## Tuning kA...')
    with control.run_robot():
        kA = differential_evolution(
            func=lambda params: cost_kA(control, robot, params[0],params[1]),
            bounds=[(0,0.5),(0,0.5)],
            strategy='best1bin', # Standard mutation strategy
            popsize=10,          # Population multiplier (total population = 10 * 2 params = 20)
            maxiter=300,          # Maximum generations to prevent infinite loops
            tol=0.01,            # Relative tolerance for convergence
            workers=1,          # Set to -1 to run simulations in parallel across all CPU cores
            seed=42
        )
    print(f'best {kA=}')
