from wpimath.geometry import Pose3d, Rotation3d
from commands2 import Subsystem
import ntcore
from typing import override
import numpy as np


class VisionServer(Subsystem):
    def __init__(self, pose_estimator):
        print("INFO: Initializing vision server")
        self.pose_estimator = pose_estimator
        self.pose_sub = ntcore.NetworkTableInstance.getDefault().getTable("Vision").getDoubleArrayTopic("robot_pose").subscribe([], ntcore.PubSubOptions(pollStorage=20, keepDuplicates=True))
    
    @override
    def periodic(self):
        for update in self.pose_sub.readQueue():
            timestamp, camera_id, apriltag_set_number, *data = update.value
            pose = np.array(data[:16], dtype=np.float64).reshape((4,4), order='F')  # order='F' specifies column-major order, which is how cpp Eigen library stores it by default and is therefore  how it is serialized on the raspberry pi side
            uncertainty = np.array(data[16:], dtype=np.float64)

            self.pose_estimator.addVisionMeasurement(
                Pose3d.from_matrix(pose),  # Im not fully sure this is the right way to construct the pose
                timestamp,
                tuple(uncertainty[:3])+(uncertainty[3:].mean(),)  # Is mean the right operation here?
            )
