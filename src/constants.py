from typing import Final

from wpimath.geometry import Pose3d, Rotation3d, Translation3d

import numpy as np

SPEED_SCALAR: Final[float] = 0.10
STARTING_POSE: Final[Pose3d] = Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))  # TODO: Find correct starting pose

ROBOT_POSES_IN_CAMERA = np.array([
        [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 0]],
        [[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 0]]
    ])