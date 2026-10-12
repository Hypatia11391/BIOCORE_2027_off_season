from src.constants import STARTING_POSE
from tests.constant_tuning.optimization.binary_search import score_binary_search

FPS = 50
kA_TOLERANCE = 0.000001

def score_kA(control, robot, kA):
    print(f'Scoring {kA=}...')
    
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

    score = -robot.robot_container.drive.get_pose_rms()
    print(f'done. {score=}')
    return score

def test_tune_simulation_constants(control, robot):
    print('############## Tuning kA...')
    with control.run_robot():
        kA = score_binary_search(low=0, high=0.5, tolerance=kA_TOLERANCE, score_fn=lambda kA: score_kA(control, robot, kA))
    print(f'best {kA=}')
