import wpilib
import hal
import hal.simulation
from wpilib.simulation import DriverStationSim

from src.constants import STARTING_POSE

FPS = 50
kA_TOLERANCE = 0.000001

def score_kA(control, robot, kA):
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

    return -robot.robot_container.drive.get_pose_rms()

def binary_search_kA(control, robot):
    low = 0
    high = 1
    do_low = do_high = True

    while high-low > kA_TOLERANCE:
        print(f'{low=}')
        print(f'{high=}')
        
        if do_low:
            low_score = score_kA(control, robot, low)
            print(f'{low_score=}')

        if do_high:
            high_score = score_kA(control, robot, high)
            print(f'{high_score=}')

        do_low = do_high = True
        mid = (low+high)/2
        if high_score>low_score:
            low = mid
            do_high = False
        else:
            high = mid
            do_low = False

    return (low+high)/2

def test_tune_simulation_constants(control, robot):
    print('############## Tuning kA...')
    with control.run_robot():
        kA = binary_search_kA(control, robot)
    print(f'best {kA=}')
