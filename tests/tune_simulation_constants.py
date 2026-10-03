import wpilib
import hal
import hal.simulation
from wpilib.simulation import DriverStationSim

FPS = 50
kA_TOLERANCE = 0.000001

def score_kA(control, robot, kA):
    hal.simulation.resetAllSimData() 
        
    # DriverStationSim.setDsAttached(True)
    # DriverStationSim.setEnabled(True)
    
    # for _ in range(1*FPS):
    #     control.step_timing(
    #         seconds=1/FPS,
    #         autonomous=False,
    #         enabled=True,
    #     )
    
    robot.robot_container.drive._init_simulation(kA=kA)
    
    for _ in range(15*FPS):
        control.step_timing(
            seconds=1/FPS,
            autonomous=True,
            enabled=True,
        )

    return -robot.robot_container.drive.get_pose_rms()

def binary_search_kA(control, robot):
    low = 0
    high = 1

    while high-low > kA_TOLERANCE:
        print("####################ITERATION")
        
        print(f'{low=}')
        print(f'{high=}')
        
        low_score = score_kA(control, robot, low)
        high_score = score_kA(control, robot, high)
        print(f'                                                                        {low_score=}')
        print(f'                                                                        {high_score=}')

        mid = (low+high)/2
        if high_score>low_score:
            low = mid
        else:
            high = mid

    return mid

def test_tune_simulation_constants(control, robot):
    print('############## Tuning kA...')
    with control.run_robot():
        #kA = binary_search_kA(control, robot)
        kA = score_kA(control, robot, None)
    print(f'best {kA=}')
