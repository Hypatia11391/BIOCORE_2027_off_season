from typing import Final

from wpimath.geometry import Pose3d, Rotation3d, Translation3d

SPEED_SCALAR: Final[float] = 1
STARTING_POSE: Final[Pose3d] = Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))  # TODO: Find correct starting pose
